// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ue-info-ai.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerUeInfoAi");

std::vector<NrMacSchedulerUeInfoAi::LcObservation>
NrMacSchedulerUeInfoAi::GetDlObservation()
{
    NS_LOG_FUNCTION(this);
    std::vector<NrMacSchedulerUeInfoAi::LcObservation> observations;
    for (const auto& ueLcg : m_dlLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();
        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            NrMacSchedulerUeInfoAi::LcObservation lcObservation = {
                m_rnti,
                lcId,
                LCPtr->m_fiveQi,
                LCPtr->m_priority,
                LCPtr->m_rlcTransmissionQueueHolDelay,
                static_cast<float>(m_dlCqi.m_wbCqi),
                static_cast<float>(LCPtr->m_rlcTransmissionQueueSize),
                static_cast<float>(m_avgTputDl),
                static_cast<float>(m_potentialTputDl)};
            observations.push_back(lcObservation);
        }
    }
    return observations;
}

std::vector<NrMacSchedulerUeInfoAi::LcObservation>
NrMacSchedulerUeInfoAi::GetUlObservation()
{
    NS_LOG_FUNCTION(this);
    std::vector<NrMacSchedulerUeInfoAi::LcObservation> observations;
    for (const auto& ueLcg : m_ulLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();
        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            NrMacSchedulerUeInfoAi::LcObservation lcObservation = {
                m_rnti,
                lcId,
                LCPtr->m_fiveQi,
                LCPtr->m_priority,
                LCPtr->m_rlcTransmissionQueueHolDelay,
                static_cast<float>(m_ulCqi.m_wbCqi),
                static_cast<float>(LCPtr->m_rlcTransmissionQueueSize),
                static_cast<float>(m_avgTputUl),
                static_cast<float>(m_potentialTputUl)};
            observations.push_back(lcObservation);
        }
    }
    return observations;
}

void
NrMacSchedulerUeInfoAi::UpdateDlWeights(NrMacSchedulerUeInfoAi::Weights& weights)
{
    m_weightsDl = weights;
}

void
NrMacSchedulerUeInfoAi::UpdateUlWeights(NrMacSchedulerUeInfoAi::Weights& weights)
{
    m_weightsUl = weights;
}

float
NrMacSchedulerUeInfoAi::GetDlReward()
{
    float reward = 0.0;
    for (const auto& ueLcg : m_dlLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            if (m_avgTputDl == 0 || LCPtr->m_rlcTransmissionQueueHolDelay == 0)
            {
                continue;
            }
            reward += std::pow(m_potentialTputDl, m_alpha) /
                      (std::max(1E-9, m_avgTputDl) * LCPtr->m_priority *
                       LCPtr->m_rlcTransmissionQueueHolDelay);
        }
    }

    return reward;
}

float
NrMacSchedulerUeInfoAi::GetUlReward()
{
    float reward = 0.0;
    for (const auto& ueLcg : m_ulLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            if (m_avgTputUl == 0 || LCPtr->m_rlcTransmissionQueueHolDelay == 0)
            {
                continue;
            }
            reward += std::pow(m_potentialTputUl, m_alpha) /
                      (std::max(1E-9, m_avgTputUl) * LCPtr->m_priority *
                       LCPtr->m_rlcTransmissionQueueHolDelay);
        }
    }

    return reward;
}

float
NrMacSchedulerUeInfoAi::GetDlRewardLyapunov(float lambda)
{
    float reward = 0.0f;
    for (const auto& ueLcg : m_dlLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();
        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            if (LCPtr->m_fiveQi <= 4)
            {
                // URLLC flow: log-bounded delay penalty
                // log1p(ratio^2) is monotonic but bounded: at 30x PDB → ~6.8
                float pdb = static_cast<float>(LCPtr->m_delayBudget.GetMilliSeconds());
                float delay = static_cast<float>(LCPtr->m_rlcTransmissionQueueHolDelay);
                float ratio = delay / std::max(pdb, 1.0f);
                reward -= lambda * static_cast<float>(std::log1p(ratio * ratio));
            }
            else
            {
                // eMBB flow: log(1+avgTput) for proportional fairness
                // Gradient = 1/(1+x), strongest at low throughput → fairness
                reward += static_cast<float>(std::log1p(std::max(0.0, m_avgTputDl)));
            }
        }
    }
    return reward;
}

float
NrMacSchedulerUeInfoAi::GetUlRewardLyapunov(float lambda)
{
    float reward = 0.0f;
    for (const auto& ueLcg : m_ulLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();
        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            if (LCPtr->m_fiveQi <= 4)
            {
                // URLLC flow: log-bounded delay penalty
                float pdb = static_cast<float>(LCPtr->m_delayBudget.GetMilliSeconds());
                float delay = static_cast<float>(LCPtr->m_rlcTransmissionQueueHolDelay);
                float ratio = delay / std::max(pdb, 1.0f);
                reward -= lambda * static_cast<float>(std::log1p(ratio * ratio));
            }
            else
            {
                // eMBB flow: log(1+avgTput) for proportional fairness
                reward += static_cast<float>(std::log1p(std::max(0.0, m_avgTputUl)));
            }
        }
    }
    return reward;
}

} // namespace ns3
