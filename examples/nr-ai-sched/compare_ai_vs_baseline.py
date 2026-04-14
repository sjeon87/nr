#!/usr/bin/env python3
# Copyright (c) 2026 University of Moratuwa
# Author: Nipuna Dulara
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only
"""
Compare the AI scheduler (msg-interface / shared memory) against
baseline non-AI schedulers (QoS, PF, RR).

This script:
  1. Runs QoS, PF, RR schedulers as single-process simulations
  2. Launches the AI scheduler as a two-process pipeline:
     - C++ simulation  (nr-ai-sched-msg --schedulerType=Ai)
     - Python PPO agent (nr_ai_sched_ppo.py)
  3. Parses FlowMonitor output from all runs
  4. Prints side-by-side comparison tables with fairness metrics

Prerequisites:
  - Build:  cd cmake-cache && ninja -j7 nr-ai-sched-msg ns3ai_nr_sched_py
  - The PPO agent requires torch and numpy:
      pip install torch numpy

Usage (from ns-3 root):
    python3 contrib/nr/examples/nr-ai-sched/compare_ai_vs_baseline.py
    python3 contrib/nr/examples/nr-ai-sched/compare_ai_vs_baseline.py --ueNum 6 --simTime 2000ms
    python3 contrib/nr/examples/nr-ai-sched/compare_ai_vs_baseline.py --baselines Qos,PF
    python3 contrib/nr/examples/nr-ai-sched/compare_ai_vs_baseline.py --skip-ai  # baselines only
"""

import argparse
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional


# ── Data classes ──────────────────────────────────────────────────

@dataclass
class FlowResult:
    flow_id: int = 0
    flow_type: str = ""
    tx_packets: int = 0
    tx_bytes: int = 0
    tx_offered: float = 0.0
    rx_bytes: int = 0
    rx_packets: int = 0
    throughput: float = 0.0
    mean_delay: float = 0.0
    mean_jitter: float = 0.0


@dataclass
class SimResult:
    scheduler: str = ""
    scenario: str = ""
    flows: list = field(default_factory=list)
    mean_throughput: float = 0.0
    mean_delay: float = 0.0
    wall_clock_s: float = 0.0


# ── Parser ────────────────────────────────────────────────────────

def parse_output(text, scheduler, scenario):
    """Parse FlowMonitor text output into a SimResult."""
    result = SimResult(scheduler=scheduler, scenario=scenario)
    current_flow = None

    for line in text.splitlines():
        line = line.strip()

        m = re.match(r"Flow (\d+) \(.*\) proto (\w+)", line)
        if m:
            if current_flow:
                result.flows.append(current_flow)
            current_flow = FlowResult(flow_id=int(m.group(1)))
            continue

        if line.startswith("Mean flow throughput:"):
            if current_flow:
                result.flows.append(current_flow)
                current_flow = None
            result.mean_throughput = float(line.split(":")[1].strip())
            continue
        if line.startswith("Mean flow delay:"):
            result.mean_delay = float(line.split(":")[1].strip())
            continue

        if current_flow is None:
            continue

        if line.startswith("Flow Type:"):
            current_flow.flow_type = line.split(":", 1)[1].strip()
        elif line.startswith("Tx Packets:"):
            current_flow.tx_packets = int(line.split(":")[1].strip())
        elif line.startswith("Tx Bytes:"):
            current_flow.tx_bytes = int(line.split(":")[1].strip())
        elif line.startswith("TxOffered:"):
            current_flow.tx_offered = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Rx Bytes:"):
            current_flow.rx_bytes = int(line.split(":")[1].strip())
        elif line.startswith("Rx Packets:"):
            current_flow.rx_packets = int(line.split(":")[1].strip())
        elif line.startswith("Throughput:"):
            current_flow.throughput = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Mean delay:"):
            current_flow.mean_delay = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Mean jitter:"):
            current_flow.mean_jitter = float(line.split(":")[1].strip().split()[0])

    if current_flow:
        result.flows.append(current_flow)
    return result


# ── Binary & environment finders ─────────────────────────────────

def find_ns3_root():
    """Walk up from script location to find ns-3 root."""
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "ns3").exists() and (p / "build").exists():
            return p
        p = p.parent
    return Path.cwd()


def find_binary(ns3_root, pattern="*nr-ai-sched-msg*"):
    """Find the simulation binary by glob pattern."""
    for p in sorted((ns3_root / "build").rglob(pattern)):
        if p.is_file() and p.stat().st_mode & 0o111:
            return p
    return None


def find_python(ns3_root):
    """Find the Python interpreter that matches the pybind11 module."""
    so_files = list((ns3_root / "contrib" / "nr" / "examples" / "nr-ai-sched")
                    .glob("ns3ai_nr_sched_py*.so"))
    if not so_files:
        # Also check ns-3 root
        so_files = list(ns3_root.glob("ns3ai_nr_sched_py*.so"))
    if so_files:
        name = so_files[0].name
        m = re.search(r"cpython-(\d)(\d+)", name)
        if m:
            major, minor = m.group(1), m.group(2)
            for candidate in [
                f"/opt/homebrew/opt/python@{major}.{minor}/bin/python{major}.{minor}",
                f"/usr/local/bin/python{major}.{minor}",
                f"python{major}.{minor}",
                "python3",
            ]:
                try:
                    subprocess.run([candidate, "--version"],
                                   capture_output=True, timeout=5)
                    return candidate
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    continue
    return "python3"


def build_pythonpath(ns3_root):
    """
    Build PYTHONPATH entries so the PPO agent can find ns3ai_nr_sched_py.

    The .so file may live in:
      - contrib/nr/examples/nr-ai-sched/
      - ns-3 root directory
    """
    paths = set()
    example_dir = ns3_root / "contrib" / "nr" / "examples" / "nr-ai-sched"
    if list(example_dir.glob("ns3ai_nr_sched_py*.so")):
        paths.add(str(example_dir))
    if list(ns3_root.glob("ns3ai_nr_sched_py*.so")):
        paths.add(str(ns3_root))
    existing = os.environ.get("PYTHONPATH", "")
    if existing:
        paths.add(existing)
    return ":".join(paths)


def check_pybind_module(ns3_root, python_bin):
    """Verify the pybind11 module can be imported."""
    pypath = build_pythonpath(ns3_root)
    env = {**os.environ, "PYTHONPATH": pypath}
    try:
        result = subprocess.run(
            [python_bin, "-c", "import ns3ai_nr_sched_py; print('OK')"],
            capture_output=True, text=True, timeout=10, env=env,
        )
        return result.returncode == 0 and "OK" in result.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


# ── Runners ───────────────────────────────────────────────────────

def run_baseline(binary, scheduler, ue_num, sim_time, scenario_id, ofdma):
    """Run a non-AI scheduler (single process)."""
    cmd = [
        str(binary),
        f"--schedulerType={scheduler}",
        f"--ueNum={ue_num}",
        f"--simTime={sim_time}",
        f"--priorityTrafficScenario={scenario_id}",
        f"--enableOfdma={'1' if ofdma else '0'}",
        f"--simTag=cmp_{scheduler}_{scenario_id}",
        "--outputDir=./",
    ]
    scenario_name = "saturation" if scenario_id == 0 else "medium-load"
    label = f"{scheduler}/{scenario_name}"
    print(f"  Running {label:<25s} ... ", end="", flush=True)

    t0 = time.time()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        elapsed = time.time() - t0
        if proc.returncode != 0:
            print(f"FAILED (exit {proc.returncode})")
            return None
        print(f"OK  ({elapsed:.1f}s)")
        result = parse_output(proc.stdout, scheduler, scenario_name)
        result.wall_clock_s = elapsed
        return result
    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        return None


def run_ai_msg(binary, ue_num, sim_time, scenario_id, ofdma,
               python_bin, agent_script, ns3_root, agent_timeout=300):
    """
    Run the AI scheduler via msg-interface (two-process pipeline).

    Pipeline:
      1. Start C++ simulation in background (creates shared memory)
      2. Wait 2s for shared memory segment creation
      3. Start Python PPO agent (attaches to shared memory)
      4. Wait for C++ to finish (it drives the simulation clock)
      5. Clean up the Python agent process
    """
    scenario_name = "saturation" if scenario_id == 0 else "medium-load"
    label = f"Ai-msg/{scenario_name}"
    print(f"  Running {label:<25s} ... ", end="", flush=True)

    # C++ command
    cpp_cmd = [
        str(binary),
        "--schedulerType=Ai",
        f"--ueNum={ue_num}",
        f"--simTime={sim_time}",
        f"--priorityTrafficScenario={scenario_id}",
        f"--enableOfdma={'1' if ofdma else '0'}",
        f"--simTag=cmp_Ai_msg_{scenario_id}",
        "--outputDir=./",
    ]

    # Python PPO agent command
    py_cmd = [
        python_bin, str(agent_script),
        "--num_flows=64",
        "--update_interval=500",
        f"--log_dir=./nr_ai_sched_logs_cmp_{scenario_id}",
    ]

    # Environment with PYTHONPATH for pybind11 module
    pypath = build_pythonpath(ns3_root)
    py_env = {**os.environ, "PYTHONPATH": pypath}

    cpp_proc = None
    py_proc = None
    t0 = time.time()

    try:
        # STEP 1: Start C++ simulation (creates shared memory segment)
        cpp_proc = subprocess.Popen(
            cpp_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            cwd=str(ns3_root),
        )

        # STEP 2: Wait for C++ to create the shared memory segment
        time.sleep(2)

        if cpp_proc.poll() is not None:
            stderr = cpp_proc.stderr.read()
            print(f"FAILED (C++ exited early: {cpp_proc.returncode})")
            if stderr:
                # Show only the most relevant error line
                for line in stderr.strip().splitlines():
                    if "msg=" in line or "ERROR" in line or "FATAL" in line:
                        print(f"    {line.strip()[:200]}")
                        break
            return None

        # STEP 3: Start Python PPO agent (connects to shared memory)
        py_proc = subprocess.Popen(
            py_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            cwd=str(agent_script.parent),
            env=py_env,
        )

        # Brief pause to let agent attach
        time.sleep(1)

        # Check if agent started successfully
        if py_proc.poll() is not None:
            py_stderr = py_proc.stderr.read()
            print(f"FAILED (Agent exited early: {py_proc.returncode})")
            if py_stderr:
                print(f"    {py_stderr.strip()[:200]}")
            # Kill C++ since agent died
            cpp_proc.kill()
            cpp_proc.wait(timeout=5)
            return None

        # STEP 4: Wait for C++ to finish (it drives the simulation clock)
        cpp_stdout, cpp_stderr = cpp_proc.communicate(timeout=agent_timeout)
        elapsed = time.time() - t0

        # STEP 5: Clean up Python agent
        if py_proc.poll() is None:
            py_proc.terminate()
            try:
                py_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                py_proc.kill()
                py_proc.wait(timeout=5)

        if cpp_proc.returncode != 0:
            print(f"FAILED (C++ exit {cpp_proc.returncode})")
            if cpp_stderr:
                for line in cpp_stderr.strip().splitlines():
                    if "msg=" in line or "ERROR" in line:
                        print(f"    {line.strip()[:200]}")
                        break
            return None

        print(f"OK  ({elapsed:.1f}s)")
        result = parse_output(cpp_stdout, "Ai-msg", scenario_name)
        result.wall_clock_s = elapsed

        # Also try reading from the output file if parsing stdout failed
        if not result.flows:
            outfile = ns3_root / f"cmp_Ai_msg_{scenario_id}"
            if outfile.exists():
                result = parse_output(outfile.read_text(), "Ai-msg", scenario_name)
                result.wall_clock_s = elapsed

        return result

    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        for p in [cpp_proc, py_proc]:
            if p and p.poll() is None:
                p.kill()
                try:
                    p.wait(timeout=5)
                except Exception:
                    pass
        return None
    except Exception as e:
        print(f"ERROR: {e}")
        for p in [cpp_proc, py_proc]:
            if p and p.poll() is None:
                p.kill()
                try:
                    p.wait(timeout=5)
                except Exception:
                    pass
        return None


# ── Fairness metrics ──────────────────────────────────────────────

def jains_fairness(values):
    if not values or all(v == 0 for v in values):
        return 0.0
    n = len(values)
    return (sum(values) ** 2) / (n * sum(v ** 2 for v in values))


def compute_metrics(result):
    if not result or not result.flows:
        return {}
    tputs = [f.throughput for f in result.flows]
    delays = [f.mean_delay for f in result.flows]
    embb_tputs = [f.throughput for f in result.flows if "eMBB" in f.flow_type]
    urllc_tputs = [f.throughput for f in result.flows if "URLLC" in f.flow_type]
    urllc_delays = [f.mean_delay for f in result.flows if "URLLC" in f.flow_type]
    return {
        "mean_tput": sum(tputs) / len(tputs) if tputs else 0,
        "mean_delay": sum(delays) / len(delays) if delays else 0,
        "jains_tput": jains_fairness(tputs),
        "total_tput": sum(tputs),
        "max_delay": max(delays) if delays else 0,
        "min_tput": min(tputs) if tputs else 0,
        "max_tput": max(tputs) if tputs else 0,
        "embb_mean_tput": sum(embb_tputs) / len(embb_tputs) if embb_tputs else 0,
        "urllc_mean_tput": sum(urllc_tputs) / len(urllc_tputs) if urllc_tputs else 0,
        "urllc_mean_delay": sum(urllc_delays) / len(urllc_delays) if urllc_delays else 0,
        "wall_clock": 0,
    }


# ── Display ───────────────────────────────────────────────────────

B = "\033[1m"
D = "\033[2m"
G = "\033[32m"
Y = "\033[33m"
R = "\033[31m"
C = "\033[36m"
U = "\033[4m"
X = "\033[0m"


def print_header(title):
    print(f"\n{B}{C}{'═' * 72}")
    print(f"  {title}")
    print(f"{'═' * 72}{X}")


def print_flow_table(results):
    if not results:
        return
    schedulers = [r.scheduler for r in results]
    n_flows = max(len(r.flows) for r in results)

    hdr = f"  {'Flow':<20s}"
    for s in schedulers:
        hdr += f" │ {B}{s:>10s}{X} Tput {s:>10s} Delay"
    print(hdr)
    sep = f"  {'─' * 20}" + ("─┼─" + "─" * 32) * len(schedulers)
    print(sep)

    for i in range(n_flows):
        ftype = results[0].flows[i].flow_type if i < len(results[0].flows) else "?"
        row = f"  {ftype:<20s}"
        for r in results:
            if i < len(r.flows):
                f = r.flows[i]
                row += f" │ {f.throughput:10.2f} Mbps {f.mean_delay:10.2f} ms"
            else:
                row += f" │ {'N/A':>10s}      {'N/A':>10s}   "
        print(row)

    print(sep)
    row = f"  {B}{'Mean':<20s}{X}"
    for r in results:
        row += f" │ {r.mean_throughput:10.2f} Mbps {r.mean_delay:10.2f} ms"
    print(row)


def print_metrics_table(results):
    all_m = [(r.scheduler, {**compute_metrics(r), "wall_clock": r.wall_clock_s})
             for r in results]
    if not all_m:
        return

    print(f"\n  {B}{'Metric':<30s}", end="")
    for name, _ in all_m:
        print(f" │ {name:>14s}", end="")
    print(X)
    print(f"  {'─' * 30}" + ("─┼─" + "─" * 14) * len(all_m))

    labels = [
        ("total_tput",      "Total Throughput (Mbps)",  "max"),
        ("mean_tput",       "Mean Throughput (Mbps)",   "max"),
        ("min_tput",        "Min Flow Tput (Mbps)",     "max"),
        ("max_tput",        "Max Flow Tput (Mbps)",     None),
        ("jains_tput",      "Jain's Fairness Index",    "max"),
        ("mean_delay",      "Mean Delay (ms)",          "min"),
        ("max_delay",       "Max Delay (ms)",           "min"),
        ("embb_mean_tput",  "eMBB Mean Tput (Mbps)",    "max"),
        ("urllc_mean_tput", "URLLC Mean Tput (Mbps)",   "max"),
        ("urllc_mean_delay","URLLC Mean Delay (ms)",    "min"),
        ("wall_clock",      "Wall-Clock Time (s)",      "min"),
    ]

    for key, label, best in labels:
        values = [m.get(key, 0) for _, m in all_m]
        best_idx = None
        if best == "max" and values:
            best_idx = values.index(max(values))
        elif best == "min" and values:
            best_idx = values.index(min(values))

        row = f"  {label:<30s}"
        for i, v in enumerate(values):
            color = G if i == best_idx else ""
            end = X if i == best_idx else ""
            row += f" │ {color}{v:>14.4f}{end}"
        print(row)


def print_fairness_analysis(results):
    print(f"\n  {B}{U}Fairness Analysis:{X}")
    for r in results:
        m = compute_metrics(r)
        if not m:
            continue
        tputs = [f.throughput for f in r.flows]
        if not tputs or min(tputs) == 0:
            continue
        ratio = max(tputs) / min(tputs)
        jfi = m["jains_tput"]
        if jfi > 0.95:
            color, verdict = G, "Fair"
        elif jfi > 0.85:
            color, verdict = Y, "Moderate"
        else:
            color, verdict = R, "Unfair"
        print(f"  {B}{r.scheduler:>10s}{X}: "
              f"Jain's={color}{jfi:.4f}{X}  "
              f"Max/Min={ratio:.2f}x  "
              f"→ {color}{verdict}{X}")


# ── Main ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare AI scheduler (msg-interface) vs baseline schedulers",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--ueNum", type=int, default=4)
    parser.add_argument("--simTime", type=str, default="1000ms")
    parser.add_argument("--ofdma", action="store_true")
    parser.add_argument("--baselines", type=str, default="Qos,PF,RR",
                        help="Comma-separated baseline schedulers")
    parser.add_argument("--scenarios", type=str, default="0,1",
                        help="0=saturation, 1=medium-load")
    parser.add_argument("--skip-ai", action="store_true",
                        help="Skip AI scheduler run (baselines only)")
    parser.add_argument("--ai-timeout", type=int, default=600,
                        help="Timeout in seconds for the AI simulation")
    parser.add_argument("--ns3-root", type=str, default=None)
    args = parser.parse_args()

    ns3_root = Path(args.ns3_root) if args.ns3_root else find_ns3_root()
    binary = find_binary(ns3_root, "*nr-ai-sched-msg*")
    if not binary:
        print(f"{R}ERROR: Cannot find nr-ai-sched-msg binary.{X}", file=sys.stderr)
        print(f"  Run: cd cmake-cache && ninja -j7 nr-ai-sched-msg && cd ..",
              file=sys.stderr)
        sys.exit(1)

    agent_script = (ns3_root / "contrib" / "nr" / "examples" /
                    "nr-ai-sched" / "nr_ai_sched_ppo.py")
    python_bin = find_python(ns3_root)

    baselines = [s.strip() for s in args.baselines.split(",")]
    scenarios = [int(s.strip()) for s in args.scenarios.split(",")]
    scenario_names = {0: "Saturation", 1: "Medium-Load"}
    mode = "OFDMA" if args.ofdma else "TDMA"

    # Pre-flight check for AI module
    ai_ready = False
    if not args.skip_ai:
        if not agent_script.exists():
            ai_status = f"{R}NOT FOUND{X}"
        elif not check_pybind_module(ns3_root, python_bin):
            ai_status = f"{Y}pybind11 module not importable{X}"
        else:
            ai_status = f"{G}READY{X}"
            ai_ready = True

    # Banner
    print(f"\n{B}{C}╔══════════════════════════════════════════════════════════════════════════╗")
    print(f"║     AI Scheduler (msg-interface)  vs  Baseline Schedulers              ║")
    print(f"╚══════════════════════════════════════════════════════════════════════════╝{X}")
    print(f"  UEs: {args.ueNum}  |  Time: {args.simTime}  |  Mode: {mode}")
    print(f"  Baselines: {', '.join(baselines)}")
    if args.skip_ai:
        print(f"  AI agent:  SKIP")
    else:
        print(f"  AI agent:  {agent_script.name}  [{ai_status}]")
    print(f"  Python:    {python_bin}")
    print(f"  Binary:    {binary.name}")

    if not args.skip_ai and not ai_ready:
        print(f"\n  {Y}⚠  AI pre-flight check failed. The AI scheduler will be skipped.{X}")
        if not agent_script.exists():
            print(f"  {Y}   Missing: {agent_script}{X}")
        else:
            print(f"  {Y}   Cannot import ns3ai_nr_sched_py. Rebuild:{X}")
            print(f"  {Y}     cd cmake-cache && ninja -j7 ns3ai_nr_sched_py{X}")

    for scenario_id in scenarios:
        sname = scenario_names.get(scenario_id, f"scenario-{scenario_id}")
        print_header(f"{sname} Scenario (priorityTrafficScenario={scenario_id})")
        print()

        results = []

        # Baselines
        for sched in baselines:
            r = run_baseline(binary, sched, args.ueNum, args.simTime,
                             scenario_id, args.ofdma)
            if r:
                results.append(r)

        # AI (msg-interface) — full two-process pipeline
        if not args.skip_ai and ai_ready:
            r = run_ai_msg(binary, args.ueNum, args.simTime, scenario_id,
                           args.ofdma, python_bin, agent_script,
                           ns3_root, args.ai_timeout)
            if r:
                results.append(r)

        if not results:
            print("  No results to display.")
            continue

        print(f"\n  {B}Per-Flow Results:{X}")
        print_flow_table(results)
        print(f"\n  {B}Summary Metrics:{X}")
        print_metrics_table(results)
        print_fairness_analysis(results)

    # Cleanup temp files
    for tag in ["cmp_*"]:
        for f in glob.glob(str(ns3_root / tag)):
            try:
                os.unlink(f)
            except OSError:
                pass

    # Cleanup agent logs
    for sc in scenarios:
        logdir = ns3_root / f"nr_ai_sched_logs_cmp_{sc}"
        if logdir.exists():
            shutil.rmtree(logdir, ignore_errors=True)

    print(f"\n{D}  Note: AI results depend on PPO training convergence.")
    print(f"  For meaningful comparison, use --simTime=5000ms or more.{X}\n")


if __name__ == "__main__":
    main()
