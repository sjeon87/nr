// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once
#include <cstdint>

namespace ns3
{

static constexpr uint32_t MAX_UES = 32; // Max UEs per slot (16 realistic, ×4 bearers aggregated per UE) + headroom

/**
 * @brief Per-UE observation written by C++ for the Python agent.
 *
 * One entry per active UE in the current scheduling slot. QoS fields are
 * aggregated across the UE's logical channels (bearers) so the agent emits a
 * single scheduling weight per UE.
 * Stored directly in shared memory so both C++ and Python can read it.
 */
struct NrSchedulerObservation {
	uint16_t rnti; 		// Radio Network Temp Identifier
	uint8_t numLcs; 	// Number of active logical channels (bearers) for this UE
	uint8_t fiveQI; 	// Representative 5QI class across the UE's bearers
	uint8_t priority; 	// Representative QoS priority level across the UE's bearers
	uint16_t holDelay; 	// Worst (max) Head of Line Delay across bearers (ms)
	float cqi; 		// Wideband CQI
	float bsr; 		// Aggregate Buffer Status Report (sum across bearers)
	float avgTput; 		// Historical Average Throughput for the UE
	float potentialTput; 	// Instantaneous achievable Throughput
	};

/**
 * @brief Per-UE action sent from the Python agent back to C++.
 *
 * The agent writes one action per active UE with a scheduling weight.
 * Higher weight means the scheduler gives that UE more resources.
*/
struct NrSchedulerAction {
	uint16_t rnti; 		// Radio Network Temp Identifier
	float weight; 		// Scheduling weight from Agent
	};

/**
 * @brief Complete message from C++ to Python each scheduling slot.
 *
 * Wraps all per-UE observations into a fixed-size block for shared memory.
 * Only the first numUes entries in the obs array are valid.
*/
struct NrSchedulerEnvMessage {			// C++ -> Python
	uint32_t numUes; 			// Active UEs per Transmission Time Interval
	float reward; 				// Reward from previous session
	bool isFinished; 			// Simulation Ended flag
	NrSchedulerObservation obs[MAX_UES]; 	// Per UE Observation array
	};

/**
 * @brief Complete message from Python to C++ each scheduling slot.
 *
 * Wraps all per-UE actions into a fixed-size block for shared memory.
 * Only the first numUes entries in the action array are valid.
*/
struct NrSchedulerActionMessage {		// Python -> C++
	uint32_t numUes; 			// Active UEs per TTI
	NrSchedulerAction action[MAX_UES]; 	// Per UE Weights
	};

} // namespace ns3

