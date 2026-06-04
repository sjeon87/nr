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

struct NrSchedulerAction {
	uint16_t rnti; 		// Radio Network Temp Identifier
	uint8_t lcID; 		// Logical Channel ID
	float weight; 		// Scheduling weight from Agent
	};

struct NrSchedulerEnvMessage {			// C++ -> Python
	uint32_t numFlows; 			// Active flows per Transmission Time Interval
	float reward; 				// Reward from previous session;
	bool isFinished; 			// Simulation Ended flag
	NrSchedulerObservation obs[MAX_FLOWS]; 	// Per flow Observation array
	};

struct NrSchedulerActionMessage {		// Python -> C++
	uint32_t numFlows; 			// Active flows per TTI
	NrSchedulerAction action[MAX_FLOWS]; 	// Per flow Weights
	};

} // namespace ns3 
