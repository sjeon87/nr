// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Anuka Hettiarachchi <anuka3382@gmail.com>

#pragma once
#include <cstdint>

namespace ns3
{

static constexpr uint32_t MAX_FLOWS = 64; // (16 UEs × 4 bearers)

/** 
 * @brief Per-flow observation written by C++ for the Python agent.
 *   
 * One entry per active flow in the current scheduling slot.         
 * Stored directly in shared memory so both C++ and Python can read it.
 */ 
struct NrSchedulerObservation {
	uint16_t rnti; 		// Radio Network Temp Identifier
	uint8_t lcID; 		// Logical Channel ID
	uint8_t fiveQI; 	// 5QI class
	uint8_t priority; 	// QoS Priority level
	uint16_t holDelay; 	// Head of Line Delay (ms)
	float cqi; 		// Wideband CQI
	float bsr; 		// Buffer Status Report
	float avgTput; 		// Historical Average Throughput
	float potentialTput; 	// Instananeous achievable Throughput
	};

/**
 * @brief Per-flow action sent from the Python agent back to C++.
 * 
 * The agent writes one action per active flow with a scheduling weight.
 * Higher weight means the scheduler gives that flow more resources.
*/
struct NrSchedulerAction {
	uint16_t rnti; 		// Radio Network Temp Identifier
	uint8_t lcID; 		// Logical Channel ID
	float weight; 		// Scheduling weight from Agent
	};

/**
 * @brief Complete message from C++ to Python each scheduling slot.
 * 
 * Wraps all per-flow observations into a fixed-size block for shared memory.
 * Only the first numFlows entries in the obs array are valid. 
*/
struct NrSchedulerEnvMessage {			// C++ -> Python
	uint32_t numFlows; 			// Active flows per Transmission Time Interval
	float reward; 				// Reward from previous session;
	bool isFinished; 			// Simulation Ended flag
	NrSchedulerObservation obs[MAX_FLOWS]; 	// Per flow Observation array
	};

/**
 * @brief Complete message from Python to C++ each scheduling slot.
 * 
 * Wraps all per-flow actions into a fixed-size block for shared memory.
 * Only the first numFlows entries in the actions array are valid.
*/
struct NrSchedulerActionMessage {		// Python -> C++
	uint32_t numFlows; 			// Active flows per TTI
	NrSchedulerAction action[MAX_FLOWS]; 	// Per flow Weights
	};

} // namespace ns3 
