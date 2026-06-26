// Copyright (c) 2026 University of Peradeniya (UoP)
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ai-ns3-msg-interface-env.h"

#ifdef HAVE_NS3_AI

#include "nr-mac-scheduler-lcg.h"

#include "ns3/abort.h"
#include "ns3/log.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerAiNs3MsgInterfaceEnv");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerAiNs3MsgInterfaceEnv);

namespace
{
/// Guards against more than one env per process.
/// A second env would share the same shared-memory channel and corrupt the handshake.
bool g_envInstanceExists = false;

/// Urgency metric for a non-GBR bearer: (1 + holDelay) / delayBudget / priority.
/// Higher is more urgent. Mirrors the NR QoS scheduler's per-LC metric.
/// A priority of 0 (unset) is treated as the least urgent (100), and a missing
/// delay budget is clamped so the division stays well-defined.
double
NonGbrUrgency(const NrMacSchedulerLC* lc)
{
    const double priority = (lc->m_priority == 0) ? 100.0 : static_cast<double>(lc->m_priority);
    const int64_t dbMs = (lc->m_delayBudget > Time(0)) ? lc->m_delayBudget.GetMilliSeconds() : 1;
    return (1.0 + static_cast<double>(lc->m_rlcTransmissionQueueHolDelay)) /
           static_cast<double>(dbMs) / priority;
}

/// Clamp a delay budget (Time) to the uint16_t milliseconds field of the struct.
uint16_t
DelayBudgetMs(const Time& delayBudget)
{
    if (delayBudget <= Time(0))
    {
        return 0;
    }
    const int64_t ms = delayBudget.GetMilliSeconds();
    return static_cast<uint16_t>(std::min<int64_t>(ms, std::numeric_limits<uint16_t>::max()));
}

} // namespace

NrMacSchedulerAiNs3MsgInterfaceEnv::NrMacSchedulerAiNs3MsgInterfaceEnv()
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(g_envInstanceExists,
                    "Only one NrMacSchedulerAiNs3MsgInterfaceEnv may exist per process: the "
                    "ns3-ai message interface is a process-wide singleton.");
    g_envInstanceExists = true;

    // The shared-memory segment is created and sized by the Python side. Here we
    // join as a non-creator (SetMemorySize would be ignored, so it is not set).
    auto interface = Ns3AiMsgInterface::Get();
    interface->SetIsMemoryCreator(false);
    interface->SetUseVector(true);
    interface->SetHandleFinish(true);
    m_msgInterface = interface->GetInterface<NrSchedulerObservation, NrSchedulerAction>();
}

NrMacSchedulerAiNs3MsgInterfaceEnv::~NrMacSchedulerAiNs3MsgInterfaceEnv()
{
    NS_LOG_FUNCTION(this);
    g_envInstanceExists = false;
}

TypeId
NrMacSchedulerAiNs3MsgInterfaceEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerAiNs3MsgInterfaceEnv")
                            .SetParent<Object>()
                            .AddConstructor<NrMacSchedulerAiNs3MsgInterfaceEnv>();
    return tid;
}

void
NrMacSchedulerAiNs3MsgInterfaceEnv::NotifyCurrentIteration(
    const std::vector<NrMacSchedulerUeInfoAi::LcObservation>&,
    bool,
    float,
    const std::string&,
    const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn& updateAllUeWeightsFn,
    const std::vector<NrMacSchedulerNs3::UePtrAndBufferReq>& ueVector)
{
    NS_LOG_FUNCTION(this << ueVector.size());

    // 1. Build and send the per-UE observations (C++ -> Python).
    m_msgInterface->CppSendBegin();
    auto* obsVec = m_msgInterface->GetCpp2PyVector();
    obsVec->resize(ueVector.size());

    for (std::size_t i = 0; i < ueVector.size(); ++i)
    {
        auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoAi>(ueVector[i].first);
        NS_ASSERT_MSG(uePtr, "UE is not a NrMacSchedulerUeInfoAi");

        NrSchedulerObservation& obs = obsVec->at(i);
        obs = NrSchedulerObservation{};
        obs.cqi = static_cast<float>(uePtr->m_dlCqi.m_wbCqi);
        obs.avgTput = static_cast<float>(uePtr->m_avgTputDl);
        obs.potentialTput = static_cast<float>(uePtr->m_potentialTputDl);
        obs.assignedBytes = uePtr->m_dlTbSize;
        obs.rnti = uePtr->m_rnti;

        // Collect the UE's active bearers, split GBR vs non-GBR.
        std::vector<NrMacSchedulerLC*> gbrLcs;
        std::vector<NrMacSchedulerLC*> nonGbrLcs;
        for (const auto& lcg : uePtr->m_dlLCG)
        {
            for (const auto lcId : lcg.second->GetActiveLCIds())
            {
                NrMacSchedulerLC* lc = lcg.second->GetLC(lcId).get();
                if (lc->m_resourceType > 0) // 1 = GBR, 2 = DC-GBR
                {
                    gbrLcs.push_back(lc);
                }
                else
                {
                    nonGbrLcs.push_back(lc);
                }
            }
        }

        // Most-urgent-first order (lc[] ordering): GBR/DC-GBR first by
        // ascending priority, then non-GBR by descending urgency metric.
        // GBR/Delay-critical-GBR bearers rank ahead of Non-GBR by their Resource
        // Type (3GPP TS 23.501 section 5.7.3.1). Within the GBR group the sort is
        // ascending Priority Level because, per 3GPP TS 23.501 section 5.7.3.3,
        // the lowest Priority Level value corresponds to the highest priority.
        // The non-GBR urgency metric (NonGbrUrgency) is an internal heuristic
        // mirroring the NR QoS scheduler, not a 3GPP-defined ordering.
        std::sort(gbrLcs.begin(), gbrLcs.end(), [](NrMacSchedulerLC* a, NrMacSchedulerLC* b) {
            return a->m_priority < b->m_priority;
        });
        std::sort(nonGbrLcs.begin(), nonGbrLcs.end(), [](NrMacSchedulerLC* a, NrMacSchedulerLC* b) {
            return NonGbrUrgency(a) > NonGbrUrgency(b);
        });

        std::vector<NrMacSchedulerLC*> ordered;
        ordered.reserve(gbrLcs.size() + nonGbrLcs.size());
        ordered.insert(ordered.end(), gbrLcs.begin(), gbrLcs.end());
        ordered.insert(ordered.end(), nonGbrLcs.begin(), nonGbrLcs.end());

        const uint8_t n =
            static_cast<uint8_t>(std::min<std::size_t>(ordered.size(), MAX_LCS_PER_UE));
        obs.numLcs = n;
        for (uint8_t k = 0; k < n; ++k)
        {
            const NrMacSchedulerLC* lc = ordered[k];
            NrSchedulerLcObservation& lo = obs.lc[k];
            lo.holDelay = lc->m_rlcTransmissionQueueHolDelay;
            lo.delayBudgetMs = DelayBudgetMs(lc->m_delayBudget);
            lo.lcId = static_cast<uint8_t>(lc->m_id);
            lo.fiveQI = lc->m_fiveQi;
            lo.priority = lc->m_priority;
            lo.resourceType = lc->m_resourceType;
            lo.bsr = static_cast<float>(lc->m_rlcTransmissionQueueSize);
        }
    }
    m_msgInterface->CppSendEnd();

    // 2. Receive the per-UE actions (Python -> C++).
    m_msgInterface->CppRecvBegin();
    std::unordered_map<uint16_t, float> rntiToWeight;
    for (const auto& act : *m_msgInterface->GetPy2CppVector())
    {
        rntiToWeight[act.rnti] = act.weight;
    }
    m_msgInterface->CppRecvEnd();

    // 3. Expand each per-UE weight across the UE's active LCs and apply.
    // The scheduler's CalculateDlWeight sums per-LC weights, so distributing
    // W / nActiveLcs makes the summed sort key equal W. Every UE in ueVector
    // gets an entry (UpdateAllUeWeights* looks up all of them); UEs the agent
    // did not return default to weight 1.0.
    NrMacSchedulerUeInfoAi::UeWeightsMap ueWeights;
    for (const auto& ue : ueVector)
    {
        auto uePtr = std::dynamic_pointer_cast<NrMacSchedulerUeInfoAi>(ue.first);
        const uint16_t rnti = uePtr->m_rnti;

        std::vector<uint8_t> activeLcs;
        for (const auto& lcg : uePtr->m_dlLCG)
        {
            for (const auto lcId : lcg.second->GetActiveLCIds())
            {
                activeLcs.push_back(lcId);
            }
        }

        float weight = 1.0f;
        if (const auto it = rntiToWeight.find(rnti); it != rntiToWeight.end())
        {
            weight = it->second;
        }

        NrMacSchedulerUeInfoAi::Weights weights;
        const double perLc =
            activeLcs.empty() ? 0.0 : static_cast<double>(weight) / activeLcs.size();
        for (const auto lcId : activeLcs)
        {
            weights[lcId] = perLc;
        }
        // NB: UeWeightsMap is keyed by uint8_t, so this truncates the RNTI to 8
        // bits and assumes RNTIs < 256. This is a deliberate deviation from the
        // standard, where the (C-)RNTI is 16-bit (3GPP TS 38.321 section 7.1,
        // range 0x0001-0xFFEF); it is safe only because the simulation uses
        // fewer than 256 UEs, and it matches the existing UpdateAllUeWeights*
        // lookup, which truncates likewise.
        ueWeights[static_cast<uint8_t>(rnti)] = weights;
    }
    updateAllUeWeightsFn(ueWeights);
}

void
NrMacSchedulerAiNs3MsgInterfaceEnv::NotifySimulationEnd()
{
    NS_LOG_FUNCTION(this);
    if (m_msgInterface)
    {
        m_msgInterface->CppSetFinished();
    }
}

} // namespace ns3

#endif // HAVE_NS3_AI