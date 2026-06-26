// Copyright (c) 2026 University of Peradeniya (UoP)
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include <cstdint>

namespace ns3
{

/**
 * @file nr-mac-scheduler-ai-msg-structs.h
 * @ingroup scheduler
 * @brief Plain-old-data message structs exchanged between the NR MAC scheduler
 *        (C++) and a Python agent through the ns3-ai message interface.
 *
 * These structs are placed byte-for-byte in shared memory (ns3-ai vector-based
 * message interface, one element per UE), so their layout is a binary contract
 * between C++ and Python: every field offset and size is verified by the
 * nr-mac-scheduler-ai-msg-structs test suite and mirrored by the pybind11
 * bindings. Do not reorder or resize fields without updating both.
 *
 * Exchange flow : one observation/action exchange happens per scheduler
 * assignment iteration (inside the resource-assignment loop, possibly many
 * times per slot), not once per slot. The Python side must simply loop
 * receive/send until the ns3-ai finish flag is set, and must not assume any
 * fixed time flow. An empty observation vector is a valid exchange (no
 * active UE in that iteration).
 *
 * Reward and episode termination are intentionally not part of these structs:
 * the reward is computed on the Python side from the observations, and
 * termination uses the native ns3-ai finish handling.
 *
 * The number of UEs per exchange is dynamic (the vector length), so no fixed
 * maximum UE count is needed. Only the per-UE bearer list is bounded.
 */

/// Maximum number of bearers (logical channels) reported per UE.
///
/// Set to 4 to match this module's UL Buffer Status Report ceiling: the Short
/// BSR (NrMacShortBsrCe) carries only 4 Logical Channel Groups (0..3), so the
/// gNB cannot distinguish more than 4 per-UE buffers in the uplink (3GPP TS
/// 38.321 section 6.1.3.1). Reporting more per-UE bearers than the BSR can resolve in
/// UL would be meaningless, so the observation array is bounded to the same 4.
static constexpr uint32_t MAX_LCS_PER_UE = 4;

/**
 * @brief Per-bearer (logical channel) observation for one UE.
 *
 * Holds the Quality-of-Service fields that are inherently per-bearer, so the
 * Python agent can see how each of a UE's traffic streams is doing rather
 * than only an aggregate. One of these is filled per active bearer of the UE.
 *
 * Downlink: all fields are valid and they come from the gNB-side RLC buffer reports.
 * Uplink: only the static QoS fields (lcId, fiveQI, priority,
 * resourceType, delayBudgetMs) and bsr are meaningful.
 * holDelay is always 0 because the standard BSR does not carry head-of-line delay.
 *
 */
struct NrSchedulerLcObservation
{
    uint16_t holDelay;      //!< Head-of-line delay of the bearer in ms
    uint16_t delayBudgetMs; //!< Packet delay budget of the bearer's 5QI in ms
    uint8_t lcId;           //!< Logical Channel identifier
    uint8_t fiveQI;         //!< 5G QoS Identifier (QoS class) of this bearer
    uint8_t priority;       //!< QoS priority level associated with the 5QI
    uint8_t resourceType;   //!< Resource type of the 5QI (non-GBR, GBR or DC-GBR)
    float bsr;              //!< Bytes waiting in this bearer 
};

/**
 * @brief Per-UE observation written by C++ for the Python agent (C++ -> Python).
 *
 * One of these is pushed per active UE into the ns3-ai message-interface
 * vector each assignment iteration, so the number of UEs is dynamic (the
 * vector length).
 *
 * The lc array holds up to MAX_LCS_PER_UE of the UE's active bearers, sorted
 * most-urgent-first: GBR/DC-GBR bearers first, then non-GBR bearers ranked by
 * (1 + holDelay) / delayBudget / priority. Bearers beyond MAX_LCS_PER_UE are
 * dropped, and only the first numLcs entries are valid. The agent still emits
 * a single scheduling weight per UE, because the NR scheduler allocates
 * resources per UE, distributing an assigned transport block among the UE's
 * bearers remains the job of the configured LC algorithm.
 *
 */
struct NrSchedulerObservation
{
    float cqi;              //!< Wideband Channel Quality Indicator of the UE
    float avgTput;          //!< Historical average throughput of the UE
    float potentialTput;    //!< Instantaneous achievable throughput of the UE
    uint32_t assignedBytes; //!< Transport block size assigned to the UE in the last
                            //!< allocation (feedback on what the previous action achieved)
    uint16_t rnti;          //!< Radio Network Temporary Identifier
    uint8_t numLcs;         //!< Number of valid entries in lc
    NrSchedulerLcObservation lc[MAX_LCS_PER_UE]; //!< Per-bearer QoS, most urgent first
};

/**
 * @brief Per-UE action sent from the Python agent back to C++ (Python -> C++).
 *
 * One of these is pushed per active UE into the ns3-ai message-interface
 * vector. Actions are matched to UEs by rnti, so the agent
 * may return them in any order. A higher weight means the scheduler gives
 * that UE more resources.
 *
 */
struct NrSchedulerAction
{
    uint16_t rnti; //!< Radio Network Temporary Identifier of the UE this weight applies to
    float weight;  //!< Scheduling weight assigned by the agent
};

} // namespace ns3
