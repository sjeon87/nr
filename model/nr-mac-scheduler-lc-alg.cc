// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-lc-alg.h"

#include "ns3/log.h"

#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerLcAlgorithm");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerLcAlgorithm);

NrMacSchedulerLcAlgorithm::NrMacSchedulerLcAlgorithm()
    : Object()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerLcAlgorithm::~NrMacSchedulerLcAlgorithm()
{
}

TypeId
NrMacSchedulerLcAlgorithm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerLcAlgorithm").SetParent<Object>();
    return tid;
}

std::map<std::pair<Lcg, LcId>, std::pair<UnassignedBytes, AssignedBytes>>
NrMacSchedulerLcAlgorithm::RetrieveActiveLcs(const std::unordered_map<uint8_t, LCGPtr>& ueLCG)
{
    NS_LOG_FUNCTION_NOARGS();
    GetSecond GetLCG;
    std::map<std::pair<Lcg, LcId>, std::pair<UnassignedBytes, AssignedBytes>> activeLc;
    for (const auto& lcg : ueLCG)
    {
        std::vector<uint8_t> lcs = GetLCG(lcg)->GetLCId();
        for (const auto& lcId : lcs)
        {
            if (GetLCG(lcg)->GetTotalSizeOfLC(lcId) > 0)
            {
                activeLc[{lcg.first, lcId}] = {GetLCG(lcg)->GetTotalSizeOfLC(lcId), 0};
            }
        }
    }
    return activeLc;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignControlBytes(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                              uint32_t tbs)
{
    std::vector<NrMacSchedulerLcAlgorithm::Assignation> ret;
    auto activeLc = RetrieveActiveLcs(ueLCG);

    if (activeLc.empty())
    {
        return ret;
    }

    for (auto& [lcId, lcData] : activeLc)
    {
        if ((lcId.second == 0 || lcId.second == 1) && (lcData.first > 0))
        {
            auto& unallocatedBytes = lcData.first;
            auto& allocatedBytes = lcData.second;
            if (unallocatedBytes <= tbs)
            {
                ret.emplace_back(lcId.first, lcId.second, unallocatedBytes);
                tbs -= unallocatedBytes;
                allocatedBytes = unallocatedBytes;
                unallocatedBytes = 0;
            }
            else
            {
                NS_LOG_WARN("TBS size " << tbs
                                        << " bytes is not sufficient for control channel LCID "
                                        << +lcId.second << " (" << unallocatedBytes << " bytes)");
            }
        }
    }
    return ret;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                           uint32_t tbs,
                                           Time slotPeriod,
                                           bool isDl) const
{
    // Work on a ueLCG copy
    std::unordered_map<uint8_t, LCGPtr> ueLCGCopy;
    for (const auto& [key, val] : ueLCG)
    {
        NrMacSchedulerLCG lcg = *val;
        ueLCGCopy[key] = std::make_unique<NrMacSchedulerLCG>(lcg);
    }

    // We first handle allocation of control logic channels. They ALWAYS have priority over data.
    auto ret = AssignControlBytes(ueLCGCopy, tbs);

    // If there was some control assignment, we need to clear it up on our ueLCG copy,
    // then pass to data
    if (!ret.empty())
    {
        // Reduce assigned control bytes from TBS
        for (const auto& assignation : ret)
        {
            tbs -= assignation.m_bytes;
            ueLCGCopy.at(assignation.m_lcg)
                ->AssignedData(assignation.m_lcId, assignation.m_bytes, isDl ? "DL" : "UL");
        }
    }

    // Allocate data for data channels using custom policies (RR, QoS, etc)
    auto retData = isDl ? DoAssignBytesToDlLC(ueLCGCopy, tbs, slotPeriod)
                        : DoAssignBytesToUlLC(ueLCGCopy, tbs);

    // Merge control and data assignments
    if (!retData.empty())
    {
        for (auto& assignment : retData)
        {
            ret.emplace_back(assignment.m_lcg, assignment.m_lcId, assignment.m_bytes);
        }
    }
    return ret;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToDlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                             uint32_t tbs,
                                             Time slotPeriod) const
{
    return AssignBytesToLC(ueLCG, tbs, slotPeriod, true);
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToUlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                             uint32_t tbs) const
{
    return AssignBytesToLC(ueLCG, tbs, Time(0), false);
}

} // namespace ns3
