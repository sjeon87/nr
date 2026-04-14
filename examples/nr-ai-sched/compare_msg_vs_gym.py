#!/usr/bin/env python3
# Copyright (c) 2026 University of Moratuwa
# Author: Nipuna Dulara
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only
"""
Compare AI scheduling approaches:
  - GSoC 2026 msg-interface (Boost shared memory IPC)
  - GSoC 2024 OpenGym       (ZMQ + protobuf IPC)

Both approaches implement PPO-based DRL scheduling for the NR MAC
layer but differ in IPC mechanism and network architecture.

This script:
  1. Runs the GSoC 2026 msg-interface pipeline (C++ + Python PPO agent)
  2. Runs the GSoC 2024 OpenGym pipeline     (Python agent drives C++)
  3. Optionally runs a baseline scheduler for reference
  4. Compares per-flow results, fairness, and wall-clock time

Prerequisites:
  - msg-interface binary + pybind module:
      cd cmake-cache && ninja -j7 nr-ai-sched-msg ns3ai_nr_sched_py
  - OpenGym binary + ns3gym Python package:
      cd cmake-cache && ninja -j7 gsoc-nr-rl-based-sched
      pip install contrib/ai/model/gym-interface/py/

Usage (from ns-3 root):
    python3 contrib/nr/examples/nr-ai-sched/compare_msg_vs_gym.py
    python3 contrib/nr/examples/nr-ai-sched/compare_msg_vs_gym.py --ueNum 4 --simTime 2000ms
    python3 contrib/nr/examples/nr-ai-sched/compare_msg_vs_gym.py --skip-gym
    python3 contrib/nr/examples/nr-ai-sched/compare_msg_vs_gym.py --skip-msg
    python3 contrib/nr/examples/nr-ai-sched/compare_msg_vs_gym.py --include-baseline Qos
"""

import argparse
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import sys
print("PYTHON USED:", sys.executable)
sys.path.append("/Users/shakir/.pyenv/versions/3.10.12/lib/python3.10/site-packages")
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
    """Parse FlowMonitor text output (same format for both examples)."""
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
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "ns3").exists() and (p / "build").exists():
            return p
        p = p.parent
    return Path.cwd()


def find_binary(ns3_root, pattern):
    for p in sorted((ns3_root / "build").rglob(pattern)):
        if p.is_file() and p.stat().st_mode & 0o111:
            return p
    return None


def find_python_for_pybind(ns3_root):
    """Find Python that matches the compiled pybind11 module."""
    so_files = list((ns3_root / "contrib" / "nr" / "examples" / "nr-ai-sched")
                    .glob("ns3ai_nr_sched_py*.so"))
    if not so_files:
        so_files = list(ns3_root.glob("ns3ai_nr_sched_py*.so"))
    if so_files:
        m = re.search(r"cpython-(\d)(\d+)", so_files[0].name)
        if m:
            major, minor = m.group(1), m.group(2)
            for candidate in [
                f"/opt/homebrew/opt/python@{major}.{minor}/bin/python{major}.{minor}",
                f"/usr/local/bin/python{major}.{minor}",
                f"python{major}.{minor}",
            ]:
                try:
                    subprocess.run([candidate, "--version"],
                                   capture_output=True, timeout=5)
                    return candidate
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    continue
    return "python3"


def build_pythonpath(ns3_root):
    """Build PYTHONPATH for the pybind11 module."""
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


def check_ns3gym():
    """Check if ns3gym Python package is installed."""
    try:
        result = subprocess.run(
            ["python3", "-c", "import ns3gym; print('OK')"],
            capture_output=True, text=True, timeout=10,
        )
        return result.returncode == 0 and "OK" in result.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


# ── Runners ───────────────────────────────────────────────────────

def run_baseline(binary, scheduler, ue_num, sim_time, scenario_id, ofdma):
    """Run a non-AI scheduler (single process, for reference)."""
    scenario_name = "saturation" if scenario_id == 0 else "medium-load"
    label = f"{scheduler}/{scenario_name}"
    print(f"  Running {label:<30s} ... ", end="", flush=True)

    cmd = [
        str(binary),
        f"--schedulerType={scheduler}",
        f"--ueNum={ue_num}",
        f"--simTime={sim_time}",
        f"--priorityTrafficScenario={scenario_id}",
        f"--enableOfdma={'1' if ofdma else '0'}",
        f"--simTag=cmp_base_{scheduler}_{scenario_id}",
        "--outputDir=./",
    ]
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        elapsed = time.time() - t0
        if proc.returncode != 0:
            print(f"FAILED (exit {proc.returncode})")
            return None
        print(f"OK  ({elapsed:.1f}s)")
        r = parse_output(proc.stdout, scheduler, scenario_name)
        r.wall_clock_s = elapsed
        return r
    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        return None


def run_ai_msg(binary, ue_num, sim_time, scenario_id, ofdma,
               python_bin, agent_script, ns3_root, timeout=600):
    """
    GSoC 2026 msg-interface: two-process pipeline.

    1. Start C++ simulation (creates shared memory segment)
    2. Wait for shared memory creation (~2s)
    3. Start Python PPO agent (attaches to shared memory)
    4. Wait for C++ to finish (it drives the simulation clock)
    5. Clean up the Python agent
    """
    scenario_name = "saturation" if scenario_id == 0 else "medium-load"
    print(f"  Running {'Ai-msg/' + scenario_name:<30s} ... ", end="", flush=True)

    cpp_cmd = [
        str(binary),
        "--schedulerType=Ai",
        f"--ueNum={ue_num}",
        f"--simTime={sim_time}",
        f"--priorityTrafficScenario={scenario_id}",
        f"--enableOfdma={'1' if ofdma else '0'}",
        f"--simTag=cmp_ai_msg_{scenario_id}",
        "--outputDir=./",
    ]
    py_cmd = [
        python_bin, str(agent_script),
        "--num_flows=64",
        "--update_interval=500",
        "--gamma=0.9",
        "--device=cpu",
        f"--log_dir=./nr_ai_sched_logs_cmp_msg_{scenario_id}",
    ]

    pypath = build_pythonpath(ns3_root)
    py_env = {**os.environ, "PYTHONPATH": pypath}

    t0 = time.time()
    cpp_proc = None
    py_proc = None

    def _kill_children():
        """Kill both child processes (safe to call multiple times)."""
        for p in [py_proc, cpp_proc]:
            if p and p.poll() is None:
                p.kill()
                try:
                    p.wait(timeout=5)
                except Exception:
                    pass

    # Clean stale shared memory from previous runs
    shm_path = Path("/tmp/boost_interprocess/NrAiSchedSeg")
    if shm_path.exists():
        shm_path.unlink()

    # Kill any orphan processes from previously interrupted runs
    try:
        result = subprocess.run(
            ["pkill", "-f", "nr-ai-sched-msg-default.*schedulerType=Ai"],
            capture_output=True, timeout=5,
        )
        result = subprocess.run(
            ["pkill", "-f", "nr_ai_sched_ppo.py"],
            capture_output=True, timeout=5,
        )
        if result.returncode == 0:
            time.sleep(1)  # Let orphans die
            # Re-clean SHM in case the killed process left it
            if shm_path.exists():
                shm_path.unlink()
    except Exception:
        pass

    try:
        # Start C++
        cpp_proc = subprocess.Popen(
            cpp_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            cwd=str(ns3_root),
        )
        time.sleep(2)

        if cpp_proc.poll() is not None:
            stderr = cpp_proc.stderr.read()
            print(f"FAILED (C++ exited early: {cpp_proc.returncode})")
            if stderr:
                for line in stderr.strip().splitlines():
                    if "msg=" in line or "ERROR" in line:
                        print(f"    {line.strip()[:200]}")
                        break
            return None

        # Start Python agent
        # Use DEVNULL for agent stdout — if piped but never drained,
        # the 64KB pipe buffer fills up and the agent's print() blocks,
        # deadlocking both processes (agent can't respond to SHM,
        # C++ blocks waiting for agent).
        py_proc = subprocess.Popen(
            py_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
            cwd=str(agent_script.parent),
            env=py_env,
        )
        time.sleep(1)

        if py_proc.poll() is not None:
            py_stderr = py_proc.stderr.read()
            print(f"FAILED (Agent exited early: {py_proc.returncode})")
            if py_stderr:
                print(f"    {py_stderr.strip()[:200]}")
            cpp_proc.kill()
            cpp_proc.wait(timeout=5)
            return None

        # Drain C++ stdout in a background thread to prevent pipe
        # buffer deadlock (same issue as agent stdout above).
        cpp_output_chunks = []
        def _drain_cpp_stdout():
            try:
                for line in cpp_proc.stdout:
                    cpp_output_chunks.append(line)
            except (ValueError, OSError):
                pass
        drain_thread = threading.Thread(target=_drain_cpp_stdout, daemon=True)
        drain_thread.start()

        # Poll both processes instead of blocking on communicate().
        # This detects Python agent crashes that would leave C++
        # deadlocked on shared memory.
        deadline = time.time() + timeout
        while time.time() < deadline:
            # Check if C++ finished (success path)
            if cpp_proc.poll() is not None:
                break

            # Check if Python agent died unexpectedly
            if py_proc.poll() is not None:
                if py_proc.returncode == 0:
                    # Agent received finish signal from C++ and exited cleanly.
                    # C++ may still be writing FlowMonitor stats — wait for it.
                    try:
                        cpp_proc.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        print("FAILED (C++ hung after agent finished)")
                        cpp_proc.kill()
                        cpp_proc.wait(timeout=5)
                        return None
                    break
                py_stderr = py_proc.stderr.read()
                print(f"FAILED (Agent died mid-run: exit {py_proc.returncode})")
                if py_stderr:
                    print(f"    {py_stderr.strip()[:300]}")
                cpp_proc.kill()
                cpp_proc.wait(timeout=5)
                return None

            time.sleep(0.5)
        else:
            # Timeout reached
            print("TIMEOUT")
            _kill_children()
            return None

        elapsed = time.time() - t0
        drain_thread.join(timeout=5)
        cpp_stdout = "".join(cpp_output_chunks)
        cpp_stderr = cpp_proc.stderr.read()

        # Clean up agent
        if py_proc.poll() is None:
            py_proc.terminate()
            try:
                py_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                py_proc.kill()
                py_proc.wait(timeout=5)

        if cpp_proc.returncode != 0:
            print(f"FAILED (C++ exit {cpp_proc.returncode})")
            return None

        print(f"OK  ({elapsed:.1f}s)")
        r = parse_output(cpp_stdout, "Ai-msg", scenario_name)
        if not r.flows:
            outfile = ns3_root / f"cmp_ai_msg_{scenario_id}"
            if outfile.exists():
                r = parse_output(outfile.read_text(), "Ai-msg", scenario_name)
        r.wall_clock_s = elapsed
        return r

    except KeyboardInterrupt:
        print("INTERRUPTED")
        _kill_children()
        raise
    except Exception as e:
        print(f"ERROR: {e}")
        _kill_children()
        return None


def run_ai_gym(ns3_root, ue_num, sim_time_s, scenario_id, ofdma,
               gym_agent, timeout=600):
    """
    GSoC 2024 OpenGym: Python agent creates ns3env which starts
    the C++ simulation via ns3gym. Single Python process.

    The gym agent script handles launching the C++ side internally
    through the ns3gym module.
    """
    scenario_name = "saturation" if scenario_id == 0 else "medium-load"
    print(f"  Running {'Ai-gym/' + scenario_name:<30s} ... ", end="", flush=True)

    cmd = [
        "python3", str(gym_agent),
        f"--ueNum={ue_num}",
        f"--simTime={sim_time_s}",
        f"--priorityTrafficScenario={scenario_id}",
        f"--enableOfdma={1 if ofdma else 0}",
        f"--simTag=cmp_ai_gym_{scenario_id}",
        "--outputDir=./",
        "--port=5559",  # Non-default port to avoid conflicts
    ]

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
            cwd=str(ns3_root)
        )
        elapsed = time.time() - t0

        if proc.returncode != 0:
            print(f"FAILED (exit {proc.returncode})")
            if "ns3gym" in (proc.stderr or ""):
                print(f"    {R}ns3gym not installed. Run:{X}")
                print(f"    pip install contrib/ai/model/gym-interface/py/")
            elif proc.stderr:
                print(f"    stderr: {proc.stderr.strip()[:200]}")
            return None

        print(f"OK  ({elapsed:.1f}s)")

        # Parse from stdout (C++ output mixed with Python output)
        result = parse_output(proc.stdout, "Ai-gym", scenario_name)
        if not result.flows:
            outfile = ns3_root / f"cmp_ai_gym_{scenario_id}"
            if outfile.exists():
                result = parse_output(outfile.read_text(), "Ai-gym", scenario_name)
        result.wall_clock_s = elapsed
        return result

    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        return None
    except Exception as e:
        print(f"ERROR: {e}")
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
    print(f"\n{B}{C}{'═' * 76}")
    print(f"  {title}")
    print(f"{'═' * 76}{X}")


def print_flow_table(results):
    if not results:
        return
    n_flows = max(len(r.flows) for r in results)

    hdr = f"  {'Flow':<20s}"
    for r in results:
        hdr += f" │ {B}{r.scheduler:>10s}{X} Tput {r.scheduler:>10s} Delay"
    print(hdr)
    sep = f"  {'─' * 20}" + ("─┼─" + "─" * 32) * len(results)
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


def print_architecture_comparison():
    """Print a static comparison table of the two IPC approaches."""
    print(f"\n  {B}{U}Architecture Comparison:{X}")
    rows = [
        ("IPC Mechanism",    "Boost shared memory",     "ZMQ + protobuf"),
        ("Latency per TTI",  "~1 µs (zero-copy)",       "~100 µs (serialize)"),
        ("Python package",   "pybind11 (ns3ai_*_py)",   "ns3gym + protobuf"),
        ("Process model",    "2 processes (C++ + Py)",   "1 process (Py→C++)"),
        ("Hidden dim",       "256",                      "64"),
        ("Input features",   "9 per flow",              "4 per flow"),
        ("Activation",       "Softplus",                "Tanh + rescale"),
        ("Advantage",        "GAE (λ=0.95)",            "Discounted returns"),
        ("URLLC QoS type",   "GBR_CONV_VOICE (5QI=1)",  "DGBR_INTER_SERV_87"),
    ]

    print(f"  {'':30s} │ {'Ai-msg (2026)':>22s} │ {'Ai-gym (2024)':>22s}")
    print(f"  {'─' * 30}─┼─{'─' * 22}─┼─{'─' * 22}")
    for label, msg_val, gym_val in rows:
        print(f"  {label:<30s} │ {msg_val:>22s} │ {gym_val:>22s}")


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
        description="Compare AI schedulers: msg-interface (2026) vs OpenGym (2024)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--ueNum", type=int, default=4)
    parser.add_argument("--simTime", type=str, default="1000ms",
                        help="Simulation time for msg-interface (ns-3 format)")
    parser.add_argument("--simTimeSec", type=float, default=1.0,
                        help="Simulation time for OpenGym (seconds)")
    parser.add_argument("--ofdma", action="store_true")
    parser.add_argument("--scenarios", type=str, default="0",
                        help="0=saturation, 1=medium-load")
    parser.add_argument("--skip-msg", action="store_true",
                        help="Skip msg-interface AI run")
    parser.add_argument("--skip-gym", action="store_true",
                        help="Skip OpenGym AI run")
    parser.add_argument("--include-baseline", type=str, default="Qos",
                        help="Include a baseline scheduler for reference (or 'none')")
    parser.add_argument("--timeout", type=int, default=600)
    parser.add_argument("--ns3-root", type=str, default=None)
    args = parser.parse_args()

    ns3_root = Path(args.ns3_root) if args.ns3_root else find_ns3_root()

    # Find binaries
    msg_binary = find_binary(ns3_root, "*nr-ai-sched-msg*")
    gym_binary = find_binary(ns3_root, "*gsoc-nr-rl-based-sched*")

    msg_agent = (ns3_root / "contrib" / "nr" / "examples" /
                 "nr-ai-sched" / "nr_ai_sched_ppo.py")
    gym_agent = (ns3_root / "contrib" / "nr" / "examples" /
                 "gsoc-nr-rl-based-sched" / "rl-sched-gym-env-ppo.py")

    python_pybind = find_python_for_pybind(ns3_root)

    scenarios = [int(s.strip()) for s in args.scenarios.split(",")]
    scenario_names = {0: "Saturation", 1: "Medium-Load"}

    # Pre-flight checks
    msg_ready = False
    gym_ready = False

    if not args.skip_msg:
        if not msg_binary:
            msg_status = f"{R}binary NOT FOUND{X}"
        elif not msg_agent.exists():
            msg_status = f"{R}agent NOT FOUND{X}"
        elif not check_pybind_module(ns3_root, python_pybind):
            msg_status = f"{Y}pybind11 module not importable{X}"
        else:
            msg_status = f"{G}READY{X}"
            msg_ready = True
    else:
        msg_status = "SKIP"

    if not args.skip_gym:
        if not gym_binary:
            gym_status = f"{R}binary NOT FOUND{X}"
        elif not gym_agent.exists():
            gym_status = f"{R}agent NOT FOUND{X}"
        elif not check_ns3gym():
            gym_status = f"{Y}ns3gym not installed{X}"
        else:
            gym_status = f"{G}READY{X}"
            gym_ready = True
    else:
        gym_status = "SKIP"

    # Banner
    print(f"\n{B}{C}╔══════════════════════════════════════════════════════════════════════════════╗")
    print(f"║   AI Scheduler Comparison: msg-interface (2026)  vs  OpenGym (2024)        ║")
    print(f"╚══════════════════════════════════════════════════════════════════════════════╝{X}")
    print(f"  UEs: {args.ueNum}  |  Mode: {'OFDMA' if args.ofdma else 'TDMA'}")
    print(f"  msg-interface: [{msg_status}]  {msg_binary.name if msg_binary else ''}")
    print(f"  OpenGym:       [{gym_status}]  {gym_binary.name if gym_binary else ''}")
    print(f"  Baseline:      {args.include_baseline}")

    # Print setup instructions for missing components
    if not args.skip_msg and not msg_ready:
        print(f"\n  {Y}⚠  msg-interface pre-flight failed:{X}")
        if not msg_binary:
            print(f"  {Y}   Build: cd cmake-cache && ninja -j7 nr-ai-sched-msg{X}")
        if msg_binary and not check_pybind_module(ns3_root, python_pybind):
            print(f"  {Y}   Build pybind: cd cmake-cache && ninja -j7 ns3ai_nr_sched_py{X}")

    if not args.skip_gym and not gym_ready:
        print(f"\n  {Y}⚠  OpenGym pre-flight failed:{X}")
        if not gym_binary:
            print(f"  {Y}   Build: cd cmake-cache && ninja -j7 gsoc-nr-rl-based-sched{X}")
            print(f"  {Y}   Note:  The CMake guard may need fixing (opengym → ai){X}")
        if gym_binary and not check_ns3gym():
            print(f"  {Y}   Install: pip install contrib/ai/model/gym-interface/py/{X}")

    # Print architecture comparison
    print_architecture_comparison()

    for scenario_id in scenarios:
        sname = scenario_names.get(scenario_id, f"scenario-{scenario_id}")
        print_header(f"{sname} Scenario")
        print()

        results = []

        # Baseline for reference
        if args.include_baseline.lower() != "none":
            ref_binary = msg_binary or gym_binary
            if ref_binary:
                r = run_baseline(ref_binary, args.include_baseline,
                                 args.ueNum, args.simTime, scenario_id, args.ofdma)
                if r:
                    results.append(r)

        # GSoC 2026: msg-interface (two-process pipeline)
        if not args.skip_msg and msg_ready:
            r = run_ai_msg(msg_binary, args.ueNum, args.simTime,
                           scenario_id, args.ofdma,
                           python_pybind, msg_agent, ns3_root, args.timeout)
            if r:
                results.append(r)

        # GSoC 2024: OpenGym (single-process via ns3gym)
        if not args.skip_gym and gym_ready:
            r = run_ai_gym(ns3_root, args.ueNum, args.simTimeSec,
                           scenario_id, args.ofdma,
                           gym_agent, args.timeout)
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
    for tag in ["cmp_base_*", "cmp_ai_msg_*", "cmp_ai_gym_*"]:
        for f in glob.glob(str(ns3_root / tag)):
            try:
                os.unlink(f)
            except OSError:
                pass

    # Cleanup agent logs
    for sc in scenarios:
        for logdir_name in [f"nr_ai_sched_logs_cmp_msg_{sc}"]:
            logdir = ns3_root / logdir_name
            if logdir.exists():
                shutil.rmtree(logdir, ignore_errors=True)

    print(f"\n{D}  Notes:")
    print(f"  - AI results depend on PPO training convergence; use longer simTime for fair comparison")
    print(f"  - Wall-clock time includes IPC overhead (the key differentiator)")
    print(f"  - The msg-interface should show lower wall-clock time due to zero-copy shared memory{X}\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Clean up SHM and any orphan processes on Ctrl+C
        shm_path = Path("/tmp/boost_interprocess/NrAiSchedSeg")
        if shm_path.exists():
            shm_path.unlink()
        subprocess.run(["pkill", "-f", "nr-ai-sched-msg-default.*schedulerType=Ai"],
                       capture_output=True, timeout=5)
        subprocess.run(["pkill", "-f", "nr_ai_sched_ppo.py"],
                       capture_output=True, timeout=5)
        print("\n  Interrupted — child processes killed, SHM cleaned.")
        sys.exit(130)
