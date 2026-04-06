// Copyright (c) 2026 University of Moratuwa
// Author: Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once
#include <cstdint>
namespace ns3 {
/**
 * @brief Maximum number of active flows supported in shared memory.
 *
 * This compile time constant caps the observation/action arrays inside
 * the shared memory envelope structs.  64 flows covers realistic NR
 * scenarios (e.g. 16 UEs × 4 bearers each).  Using a fixed size avoids
 * the complexity of Boost allocators in shared memory segments.
 */
static constexpr uint32_t MAX_FLOWS = 64;
/**
 * @brief Per-flow observation written by C++ for the Python agent.
 *
 * Each active logical channel produces one NrSchedObservation entry.
 * The struct carries the 9 : RNTI, LCID, 5QI, priority, HoL delay,
 * CQI, BSR, average throughput, and potential throughput.
 *
 * @note This is a POD type. So it can be placed directly in Boost shared
 * memory.
 */
struct NrSchedObservation {
  uint16_t rnti;       ///< Radio Network Temporary Identifier
  uint8_t lcId;        ///< Logical Channel Identifier
  uint8_t fiveQi;      ///< 5QI Class
  uint8_t priority;    ///< QoS priority level
  uint16_t holDelay;   ///< Head-of-Line delay in milliseconds
  float cqi;           ///< Wideband CQI
  float bsr;           ///< Buffer Status Report
  float avgTput;       ///< Historical average throughput (bit/symbol)
  float potentialTput; ///< Instantaneous achievable rate (bit/symbol)
};
/**
 * @brief Perflow action written by the Python agent for C++.
 *
 * After processing observations, the DRL agent writes one NrSchedAction
 * per active flow.  The weight field is used to modify the Proportional
 * Fair priority score inside the NR scheduler.
 */
struct NrSchedAction {
  uint16_t rnti; ///< Radio Network Temporary Identifier
  uint8_t lcId;  ///< Logical Channel Identifier
  float weight;  ///< Scheduling weight from the DRL agent
};
/**
 * @brief Envelope struct for C++ to Python direction.
 *
 * Placed in shared memory via Ns3AiMsgInterfaceImpl<NrSchedEnvMsg,
 * NrSchedActMsg>.  The C++ side fills numFlows entries of the obs[]
 * array and writes the scalar reward / isFinished flag before
 * signalling Python.
 */
struct NrSchedEnvMsg {
  uint32_t numFlows;                 ///< Number of active flows this TTI
  float reward;                      ///< Reward from the previous action
  bool isFinished;                   ///< True when the simulation has ended
  NrSchedObservation obs[MAX_FLOWS]; ///< Perflow observations
};
/**
 * @brief Envelope struct for Python to C++ direction (actions).
 *
 * The Python agent fills numFlows entries of the actions[] array
 * and signals C++ after it completes the forward pass.
 */
struct NrSchedActMsg {
  uint32_t numFlows; ///< Number of flows for which actions are provided
  NrSchedAction actions[MAX_FLOWS]; ///< Perflow scheduling weights
};
} // namespace ns3