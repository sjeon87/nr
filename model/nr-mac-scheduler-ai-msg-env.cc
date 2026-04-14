// Copyright (c) 2026 University of Moratuwa
// Author : Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
#include "nr-mac-scheduler-ai-msg-env.h"
#ifdef HAVE_NS3_AI
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrMacSchedulerAiMsgEnv");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerAiMsgEnv);

NrMacSchedulerAiMsgEnv::NrMacSchedulerAiMsgEnv()
{
    NS_LOG_FUNCTION(this);
    auto interface = Ns3AiMsgInterface::Get();
    interface->SetIsMemoryCreator(true);
    interface->SetUseVector(false);
    interface->SetHandleFinish(true);
    // Size the shared memory segment to hold both envelope structs
    // plus the synchronisation structure with comfortable headroom.
    interface->SetMemorySize(sizeof(NrSchedEnvMsg) + sizeof(NrSchedActMsg) + 4096);
    interface->SetNames("NrAiSchedSeg", "NrAiSchedEnv", "NrAiSchedAct", "NrAiSchedLock");
    m_msgInterface = interface->GetInterface<NrSchedEnvMsg, NrSchedActMsg>();
}

NrMacSchedulerAiMsgEnv::NrMacSchedulerAiMsgEnv(uint32_t numFlows)
    : NrMacSchedulerAiMsgEnv()
{
    NS_LOG_FUNCTION(this << numFlows);
    m_numFlows = numFlows;
}

NrMacSchedulerAiMsgEnv::~NrMacSchedulerAiMsgEnv()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrMacSchedulerAiMsgEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerAiMsgEnv")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrMacSchedulerAiMsgEnv>();
    return tid;
}

void
NrMacSchedulerAiMsgEnv::DoDispose()
{
    NS_LOG_FUNCTION(this);
    // Signal Python that the simulation is finished so that
    // the Python process can exit its receive loop cleanly.
    if (m_msgInterface != nullptr)
    {
        m_msgInterface->CppSetFinished();
    }
}

void
NrMacSchedulerAiMsgEnv::NotifyCurrentIteration(
    const std::vector<NrMacSchedulerUeInfoAi::LcObservation>& observations,
    bool isGameOver,
    float reward,
    const std::string& extraInfo,
    const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn& updateAllUeWeightsFn)
{
    NS_LOG_FUNCTION(this);
    // Pack observations into shared memory (C++ -> Python)
    m_msgInterface->CppSendBegin();
    auto* envMsg = m_msgInterface->GetCpp2PyStruct();
    envMsg->numFlows = std::min(static_cast<uint32_t>(observations.size()), MAX_FLOWS);
    envMsg->reward = reward;
    envMsg->isFinished = isGameOver;
    for (uint32_t i = 0; i < envMsg->numFlows; ++i)
    {
        envMsg->obs[i].rnti = observations[i].rnti;
        envMsg->obs[i].lcId = observations[i].lcId;
        envMsg->obs[i].fiveQi = observations[i].fiveQi;
        envMsg->obs[i].priority = observations[i].priority;
        envMsg->obs[i].holDelay = observations[i].holDelay;
        envMsg->obs[i].cqi = observations[i].cqi;
        envMsg->obs[i].bsr = observations[i].bsr;
        envMsg->obs[i].avgTput = observations[i].avgTput;
        envMsg->obs[i].potentialTput = observations[i].potentialTput;
    }
    m_msgInterface->CppSendEnd(); // Python wakes up
    // Wait for Python's action
    m_msgInterface->CppRecvBegin(); // blocks until Python sends
    auto* actMsg = m_msgInterface->GetPy2CppStruct();
    // Convert flat action array to UeWeightsMap
    NrMacSchedulerUeInfoAi::UeWeightsMap ueWeightsMap;
    for (uint32_t i = 0; i < actMsg->numFlows; ++i)
    {
        ueWeightsMap[actMsg->actions[i].rnti][actMsg->actions[i].lcId] =
            static_cast<double>(actMsg->actions[i].weight);
    }
    m_msgInterface->CppRecvEnd();
    // Apply the DRL weights to the NR scheduler
    updateAllUeWeightsFn(ueWeightsMap);
}
} // namespace ns3
#endif // HAVE_NS3_AI
