// Copyright (c) 2026 University of Peradeniya (UoP)
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#if __has_include("ns3/ai-module.h")
#define HAVE_NS3_AI

#include "nr-mac-scheduler-ai-msg-structs.h"
#include "nr-mac-scheduler-ue-info-ai.h"

#include "ns3/ai-module.h"
#include "ns3/nr-export.h"
#include "ns3/object.h"

#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup scheduler
 * @brief ns3-ai message-interface environment for the RL-based NR scheduler.
 *
 * Shared-memory counterpart of NrMacSchedulerAiNs3GymEnv. Instead of ns3-gym
 * it exchanges observations and actions with a Python agent
 * through the ns3-ai Message Interface (vector mode). It
 * implements the NrMacSchedulerUeInfoAi::NotifyCbMsg callback and is bound to a
 * NrMacSchedulerTdmaAi / NrMacSchedulerOfdmaAi through the NotifyCbDlMsg
 * attribute (or SetNotifyCbDlMsg()).
 *
 * Downlink only: NotifyCurrentIteration() reads the UE's downlink scheduler
 * state (m_dlLCG, DL CQI, DL throughput, DL TB size).
 * It would send downlink observations for an uplink
 * decision.
 *
 * Single instance per process: the ns3-ai message interface is a process-wide
 * singleton, so creating a second env would share the same shared-memory
 * channel and corrupt the handshake (the constructor aborts if one already
 * exists).
 *
 * The per-bearer QoS fields exposed in each observation (fiveQI, priority,
 * resourceType, delayBudgetMs) are the standardized 5QI QoS characteristics of
 * 3GPP TS 23.501 section 5.7 (Table 5.7.4-1): see section 5.7.2.1 (5QI),
 * 5.7.3.2 (Resource Type: Non-GBR / GBR / Delay-critical GBR), 5.7.3.3
 * (Priority Level) and 5.7.3.4 (Packet Delay Budget). The CQI is the wideband
 * Channel Quality Indicator of 3GPP TS 38.214 section 5.2.2.1.
 *
 * Per assignment iteration the scheduler calls NotifyCurrentIteration(). This
 * class fills one NrSchedulerObservation per active UE (CQI, throughput, last
 * TB size and a per-bearer lc[] array sorted most-urgent-first), performs one
 * C++->Python / Python->C++ handshake, then expands each per-UE weight returned
 * by the agent across that UE's active logical channels (W / nActiveLcs each,
 * so the scheduler's summed sort key equals W) and applies it. The reward is
 * computed on the Python side from the observations; termination uses the
 * native ns3-ai finish flag (NotifySimulationEnd()).
 *
 * Process model: the Python side is the shared-memory creator and launches the
 * ns-3 program; this class joins the segment as a non-creator.
 *
 * @see NrMacSchedulerAiNs3GymEnv
 * @see NrSchedulerObservation
 */
class NR_EXPORT NrMacSchedulerAiNs3MsgInterfaceEnv : public Object
{
  public:
    /**
     * @brief Constructor.
     * Configures the ns3-ai vector message interface and
     * joins the shared-memory segment created by the Python side.
     */
    NrMacSchedulerAiNs3MsgInterfaceEnv();

    /**
     * @brief Destructor.
     */
    ~NrMacSchedulerAiNs3MsgInterfaceEnv() override;

    /**
     * @brief GetTypeId
     * @return The TypeId of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief NotifyCbMsg implementation: one observation/action exchange.
     * @param observations the compact per-LC observations (unused, the full state is read from
     * ueVector instead)
     * @param isGameOver unused (termination via the ns3-ai finish flag)
     * @param reward unused (the reward is computed on the Python side)
     * @param extraInfo unused
     * @param updateAllUeWeightsFn callback that applies the per-UE weights map
     * @param ueVector the active UEs of the current assignment iteration
     */
    void NotifyCurrentIteration(
        const std::vector<NrMacSchedulerUeInfoAi::LcObservation>& observations,
        bool isGameOver,
        float reward,
        const std::string& extraInfo,
        const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn& updateAllUeWeightsFn,
        const std::vector<NrMacSchedulerNs3::UePtrAndBufferReq>& ueVector);

    /**
     * @brief Signal the end of the simulation to the Python agent.
     *
     * Sets the native ns3-ai finish flag so the Python loop can break.
     */
    void NotifySimulationEnd();

  private:
    /// The ns3-ai vector-mode message interface (process-wide singleton).
    Ns3AiMsgInterfaceImpl<NrSchedulerObservation, NrSchedulerAction>* m_msgInterface{nullptr};
};

} // namespace ns3

#endif // HAVE_NS3_AI
