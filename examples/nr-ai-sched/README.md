# NR AI Scheduler Example (msg-interface)
This example demonstrates AI-driven MAC scheduling in 5G NR using the
`ns-3-ai` msg-interface (Boost shared memory IPC).
## Architecture

**[ ns-3 Environment ]** `nr-ai-sched-msg (C++)` -> `NrMacSchedulerAiMsgEnv`
      ↓ ↑
      ↓ ↑ *(boost::ipc shared memory)*
      ↓ ↑
      * `NrSchedEnvMsg` (Sends perflow observations and rewards)
      * `NrSchedActMsg` (Receives scheduling weights)
      ↓ ↑
      ↓ ↑
**[ Python RL Agent ]**
`ns3ai_nr_sched_py` -> `nr_ai_sched_ppo.py (PyTorch PPO)`
## Prerequisites
- ns-3 (3.42+) with `contrib/nr` and `contrib/ai`
- Boost (for shared memory)
- Python 3.10+ with `torch` and `numpy`
- pybind11 (for the Python bindings module)
Install Python dependencies:
    pip install torch numpy pybind11
## Build
    ./ns3 configure --enable-examples --enable-tests
    ./ns3 build
## Run
**Terminal 1 — C++ simulation:**
    ./ns3 run "nr-ai-sched-msg --ueNum=4 --simTime=2000ms"
**Terminal 2 — Python PPO agent (start AFTER C++):**
    cd contrib/nr/examples/nr-ai-sched/
    python3 nr_ai_sched_ppo.py
> **Important:** The C++ process creates the shared memory segment.
> Start the Python agent **after** the C++ simulation is running.
## CLI Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--ueNum` | 4 | Number of UEs |
| `--simTime` | 2000ms | Simulation duration |
| `--enableOfdma` | 0 | 0=TDMA, 1=OFDMA |
| `--schedulerType` | Ai | PF, RR, Qos, or Ai |
| `--priorityTrafficScenario` | 0 | 0=saturation, 1=medium-load |
| `--bandwidth` | 10MHz | System bandwidth |
## Traffic Setup
- **Even UEs:** 1 flow (eMBB, NGBR_LOW_LAT_EMBB, 5QI=9)
- **Odd UEs:** 2 flows (eMBB + URLLC via GBR_CONV_VOICE, 5QI=1)
## Output
FlowMonitor statistics are printed to stdout and written to
`<outputDir>/<simTag>`.