# Copyright (c) 2026 University of Peradeniya (UoP)
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

"""Proportional Fair scheduler agent driving the NR MAC scheduler over the ns3-ai
message interface.

Launches gsoc-nr-ai-sched.cc with --ueLevelSchedulerType=Ai and reimplements the
built-in C++ PF scheduler (NrMacSchedulerUeInfoPF::CompareUeWeightsDl) in
Python, computing each UE's scheduling weight from the shared-memory
observation:

    weight = potentialTput**alpha / max(1e-9, avgTput)

Both inputs come straight from the observation struct, so agreement with the
built-in scheduler is evidence that the observation carries the full state the
C++ PF metric uses. This is the same experiment as gsoc-nr-ai-sched-qos.py, one
level simpler: the QoS weight is this metric summed over the UE's bearers and
scaled by (100 - priority) and the delay budget factor.

Equivalence check (the FlowMonitor summary must come out byte-identical):

    python3 gsoc-nr-ai-sched-pf.py --ueNum 2 --simTag pf-agent
    ../../../../ns3 run "gsoc-nr-ai-sched --ueLevelSchedulerType=PF --ueNum=2 --simTag=pf-builtin"

Verified byte-identical for TDMA with 2 UEs and for OFDMA with 4 UEs, both in
the saturation traffic scenario.

Run from this directory with the interpreter the bindings were built against
(see the cpython tag on ns3ai_nr_sched_py.*.so):
    python3 gsoc-nr-ai-sched-pf.py --ueNum 2 --simTag pf-run
"""

import argparse
import glob
import os
import subprocess
import sys
import time
import traceback

# Make the binding (.so dropped next to this file) and ns3ai_utils importable.
# nr and ns3-ai can each be installed under contrib/ or src/; both trees have
# the same layout, so this file is always four levels below the ns-3 root and
# only the module root differs. Look ns3ai_utils up in both - a path that does
# not exist is harmless on sys.path.
_HERE = os.path.dirname(os.path.abspath(__file__))
_NS3_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
_MODULE_ROOTS = ("contrib", "src")
sys.path[:0] = [_HERE] + [
    os.path.join(_NS3_ROOT, root, "ai", "python_utils") for root in _MODULE_ROOTS
]

import ns3ai_nr_sched_py as py_binding  # noqa: E402
from ns3ai_utils import Experiment  # noqa: E402

#: PF fairness exponent. Both NrMacSchedulerTdmaPF and the AI scheduler (via
#: NrMacSchedulerTdmaQos) expose it as the "FairnessIndex" attribute, which
#: defaults to 1 (traditional 3GPP PF; 0 would be round-robin in throughput).
#: The example does not change the attribute.
PF_ALPHA = 1.0

# If ns-3 dies without raising the finish flag - any
# NS_ABORT_MSG/NS_FATAL_ERROR, which ends in std::terminate() and therefore
# never unwinds the stack to run ~Ns3AiMsgInterfaceImpl(), or a hard crash -
# this driver spins forever and no Python thread, signal handler or timeout can
# ever run to notice. Only a separate process can end it, so watch the ns-3
# process from one.
_WATCHDOG = """
import os
import sys
import time

import psutil

ns3, driver = int(sys.argv[1]), int(sys.argv[2])
while psutil.pid_exists(driver):
    time.sleep(0.5)
    try:
        # A dead child of a blocked parent is never reaped, and a zombie still
        # answers pid_exists(), so the status has to be checked explicitly.
        alive = psutil.Process(ns3).status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        alive = False
    if alive:
        continue
    # ns-3 also exits before the driver on a normal shutdown, so give the
    # driver a moment to finish on its own before assuming it is stuck.
    time.sleep(5)
    if psutil.pid_exists(driver):
        print(
            "gsoc-nr-ai-sched-pf.py: ns-3 exited without raising the finish flag; the "
            "driver is blocked in PyRecvBegin(). Killing it.",
            file=sys.stderr,
        )
        # SIGKILL, not SIGTERM: Python's SIGTERM handling also needs the
        # interpreter to run bytecode, which the held GIL prevents.
        os.kill(driver, 9)
    break
"""


def _start_watchdog(exp):
    """Watch the ns-3 process so its death cannot leave this driver spinning."""
    return subprocess.Popen([sys.executable, "-c", _WATCHDOG, str(exp.proc.pid), str(os.getpid())])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    # Scenario knobs forwarded to gsoc-nr-ai-sched.cc (defaults match
    # gsoc-nr-rl-based-sched.cc so runs are comparable out of the box).
    parser.add_argument("--ueNum", type=int, default=2, help="Number of UEs")
    parser.add_argument(
        "--priorityTrafficScenario",
        type=int,
        default=0,
        help="0: saturation, 1: medium-load",
    )
    parser.add_argument("--simTime", type=str, default="1000ms", help="Simulation time (ns-3 Time)")
    parser.add_argument("--numerology", type=int, default=0, help="NR numerology")
    parser.add_argument("--centralFrequency", type=float, default=4e9, help="Frequency [Hz]")
    parser.add_argument("--bandwidth", type=float, default=10e6, help="Bandwidth [Hz]")
    parser.add_argument("--totalTxPower", type=float, default=43, help="Total TX power [dBm]")
    parser.add_argument("--simTag", type=str, default="default", help="Output filename tag")
    parser.add_argument("--outputDir", type=str, default="./", help="Output directory")
    parser.add_argument("--enableOfdma", type=int, default=0, help="1: Ofdma, 0: Tdma")
    parser.add_argument(
        "--enableLcLevelQos",
        type=int,
        default=0,
        help="1: QoS LC scheduler, 0: Round-Robin LC scheduler",
    )
    parser.add_argument("--simSeed", type=int, default=1, help="RngSeedManager run number")
    return parser.parse_args()


def pf_weight(o):
    """Reproduce NrMacSchedulerUeInfoPF::CompareUeWeightsDl from the observation.

    Unlike the QoS weight this is a pure per-UE metric: it ignores the bearer
    list entirely, which is why it needs nothing beyond the two throughput
    fields. The max(1e-9, ...) guard mirrors the C++ std::max(1E-9, m_avgTputDl)
    and is what keeps a UE that has never been served from dividing by zero.
    """
    return o.potentialTput**PF_ALPHA / max(1e-9, o.avgTput)


def main():
    args = parse_args()

    # Prefer launching the already-built executable directly (ns3ai_utils runs
    # a file path as-is). This avoids "./ns3 run" rebuilding the whole tree,
    # which is both slow and brittle if an unrelated contrib module fails to
    # build. build/ mirrors the source tree, so the binary follows nr into
    # either build/contrib/nr or build/src/nr.
    exe = [
        path
        for root in _MODULE_ROOTS
        for path in glob.glob(
            os.path.join(_NS3_ROOT, "build", root, "nr", "examples", "ns3*-gsoc-nr-ai-sched-*")
        )
    ]
    # NR_AI_SCHED_TARGET overrides the launch target (e.g. a timing wrapper
    # script around the executable, for transport benchmarks).
    target = os.environ.get("NR_AI_SCHED_TARGET") or (exe[0] if exe else "gsoc-nr-ai-sched")

    setting = {
        "ueNum": args.ueNum,
        "priorityTrafficScenario": args.priorityTrafficScenario,
        "simTime": args.simTime,
        "numerology": args.numerology,
        "centralFrequency": args.centralFrequency,
        "bandwidth": args.bandwidth,
        "totalTxPower": args.totalTxPower,
        "simTag": args.simTag,
        "outputDir": args.outputDir,
        "enableOfdma": args.enableOfdma,
        "enableLcLevelQos": args.enableLcLevelQos,
        "simSeed": args.simSeed,
        "ueLevelSchedulerType": "Ai",
    }

    exp = Experiment(
        target,
        _NS3_ROOT,
        py_binding,
        handleFinish=True,
        useVector=True,
        # This side creates the segment (ns-3 only attaches), so it sizes it.
        # One element per UE is the largest an exchange can get: C++ resizes
        # the observation vector to the UEs active in that iteration.
        vectorSize=args.ueNum,
        # A UE costs 76 bytes here (68 observation + 8 action). Measured
        # capacity is 27 UEs with the 4 KiB ns3ai_utils default and 4078 with
        # 512 KiB. Headroom is nearly free (virtual memory), and an undersized
        # segment fails obscurely, as a Boost bad_alloc when a vector grows.
        shmSize=512 * 1024,
        # Must match the C++ singleton Ns3AiMsgInterface::BuildSegmentName(),
        # which is "ns3-ai_<trialName>" with the default trialName
        # "single_trial".
        segName="ns3-ai_single_trial",
    )
    msgInterface = exp.run(setting=setting, show_output=True)
    watchdog = _start_watchdog(exp)

    exchanges = 0
    first_done = None  # perf_counter at the end of the first exchange
    last_done = None  # perf_counter at the end of the most recent exchange
    loop_t0 = time.perf_counter()
    try:
        while True:
            # Receive observations from C++.
            msgInterface.PyRecvBegin()
            if msgInterface.PyGetFinished():
                break

            # Compute the PF metric for each UE and send it back.
            msgInterface.PySendBegin()
            obs = msgInterface.GetCpp2PyVector()
            act = msgInterface.GetPy2CppVector()
            act.resize(len(obs))
            for i in range(len(obs)):
                act[i].rnti = obs[i].rnti
                act[i].weight = float(pf_weight(obs[i]))
            exchanges += 1
            msgInterface.PyRecvEnd()
            msgInterface.PySendEnd()

            # Bracket the timed region by the exchanges themselves. loop_t0 is
            # taken before the first PyRecvBegin(), which blocks until ns-3 has
            # built the scenario, and the loop only exits once C++ raises the
            # finish flag - which it does after FlowMonitor has written the
            # output file. Both ends would otherwise be charged to the
            # per-exchange figure.
            last_done = time.perf_counter()
            if first_done is None:
                first_done = last_done

    except Exception:
        print("Exception in gsoc-nr-ai-sched-pf.py:")
        traceback.print_exc()
        sys.exit(1)

    finally:
        # loop_wall keeps its original full-span meaning, so figures recorded
        # from earlier runs stay comparable; per_exchange_us is now measured
        # over the bracketed steady-state region only.
        loop_wall = time.perf_counter() - loop_t0
        timed_steps = max(exchanges - 1, 0)
        timed_wall = last_done - first_done if timed_steps else 0.0
        per_exchange_us = timed_wall / timed_steps * 1e6 if timed_steps else 0.0
        print(
            f"gsoc-nr-ai-sched-pf.py: completed {exchanges} observation/action exchanges | "
            f"MSG_RESULT steps={exchanges} loop_wall_s={loop_wall:.3f} "
            f"timed_steps={timed_steps} timed_wall_s={timed_wall:.3f} "
            f"per_exchange_us={per_exchange_us:.1f}"
        )
        watchdog.terminate()
        del exp


if __name__ == "__main__":
    main()
