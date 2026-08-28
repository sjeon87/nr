// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8.1 - UE power state event triggers

#include "nr-ue-phy-energy-listener.h"

#include "nr-ue-drx-model.h"
#include "nr-ue-energy-model.h"
#include "nr-ue-phy.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrUePhyEnergyListener");
NS_OBJECT_ENSURE_REGISTERED(NrUePhyEnergyListener);

TypeId
NrUePhyEnergyListener::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUePhyEnergyListener")
            .SetParent<Object>()
            .SetGroupName("Nr")
            .AddConstructor<NrUePhyEnergyListener>()
            .AddAttribute("UseRankAsRxChains",
                          "OPTIONAL non-3GPP approximation: treat the reported "
                          "MIMO rank as the number of powered receive chains and "
                          "re-apply the TR 38.840 Table 21 antenna scaling on each "
                          "downlink transport block. Spatial layers and RF chains "
                          "are different quantities and the spec defines no such "
                          "mapping, so this is off by default; the 3GPP behaviour "
                          "is the static NrUeEnergyModel::ActiveRxChains.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUePhyEnergyListener::m_useRankAsRxChains),
                          MakeBooleanChecker());
    return tid;
}

NrUePhyEnergyListener::NrUePhyEnergyListener()
    : m_slotDuration(MilliSeconds(1)),
      m_lastBwpMhz(0),
      m_useRankAsRxChains(false)
{
    NS_LOG_FUNCTION(this);
}

NrUePhyEnergyListener::~NrUePhyEnergyListener()
{
    NS_LOG_FUNCTION(this);
}

void
NrUePhyEnergyListener::SetPhy(Ptr<NrUePhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    NS_ASSERT_MSG(phy, "NrUePhy pointer must not be null");
    m_phy = phy;
    m_slotDuration = phy->GetSlotPeriod();
    RefreshBwpScaling();
    phy->TraceConnectWithoutContext(
        "DlDataStats",
        MakeCallback(&NrUePhyEnergyListener::DlTbReceivedCallback, this));
    phy->TraceConnectWithoutContext("UlDataStats",
                                    MakeCallback(&NrUePhyEnergyListener::UlTbSentCallback, this));
}

void
NrUePhyEnergyListener::SetEnergyModel(Ptr<NrUeEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    NS_ASSERT_MSG(model, "NrUeEnergyModel pointer must not be null");
    m_model = model;
    // Attach order (phy first or model first) must not matter.
    RefreshBwpScaling();
}

void
NrUePhyEnergyListener::SetDrxModel(Ptr<NrUeDrxModel> drx)
{
    NS_LOG_FUNCTION(this << drx);
    NS_ASSERT_MSG(drx, "NrUeDrxModel pointer must not be null");
    m_drx = drx;
}

void
NrUePhyEnergyListener::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_phy = nullptr;
    m_model = nullptr;
    m_drx = nullptr;
    Object::DoDispose();
}

void
NrUePhyEnergyListener::RefreshBwpScaling()
{
    NS_LOG_FUNCTION(this);
    if (!m_phy || !m_model)
    {
        return;
    }
    // GetChannelBandwidth() is in Hz; TR 38.840 Section 8.1.3 scales in MHz.
    uint32_t bwMhz = m_phy->GetChannelBandwidth() / 1000000;
    if (bwMhz > 0 && bwMhz != m_lastBwpMhz)
    {
        m_model->ApplyBwpScaling(bwMhz);
        m_lastBwpMhz = bwMhz;
    }
}

void
NrUePhyEnergyListener::DlTbReceivedCallback(uint64_t imsi,
                                            uint32_t tbSize,
                                            uint32_t symStart,
                                            uint32_t numSym,
                                            uint32_t rank)
{
    NS_LOG_FUNCTION(this << imsi << tbSize << symStart << numSym << rank);
    if (!m_model || tbSize == 0)
    {
        return;
    }
    if (m_model->GetCurrentState() == NR_UE_DEEP_SLEEP)
    {
        m_model->TriggerSetupTransition();
    }
    RefreshBwpScaling();
    // Table 21 scales with powered receive chains, which 5G-LENA does not model;
    // that count is a static configuration on the energy model. Rank counts
    // spatial layers, so it drives the scaling only if the user opts in.
    if (m_useRankAsRxChains && rank >= 1)
    {
        m_model->ApplyAntennaScaling(rank);
    }
    m_model->ChangeState(NR_UE_PDCCH_PDSCH);
    if (m_drx)
    {
        m_drx->NotifyDataActivity();
    }
    // TR 38.840 Table 18/20 values are averaged over the operations within a slot
    // (Release 16, p.63), so the active value is charged for the whole slot.
    Time end = Simulator::Now() + m_slotDuration;
    m_activeUntil = std::max(m_activeUntil, end);
    Simulator::Schedule(m_slotDuration, &NrUePhyEnergyListener::ReturnToMonitoring, this);
}

void
NrUePhyEnergyListener::UlTbSentCallback(uint64_t imsi,
                                        uint32_t tbSize,
                                        uint32_t symStart,
                                        uint32_t numSym,
                                        uint32_t rank)
{
    NS_LOG_FUNCTION(this << imsi << tbSize << symStart << numSym << rank);
    if (!m_model || tbSize == 0)
    {
        return;
    }
    RefreshBwpScaling();
    if (m_phy)
    {
        m_model->SetUlTxPowerDbm(m_phy->GetTxPower());
    }
    m_model->ChangeState(NR_UE_UL_TX);
    if (m_drx)
    {
        m_drx->NotifyDataActivity();
    }
    Time end = Simulator::Now() + m_slotDuration;
    m_activeUntil = std::max(m_activeUntil, end);
    Simulator::Schedule(m_slotDuration, &NrUePhyEnergyListener::ReturnToMonitoring, this);
}

void
NrUePhyEnergyListener::ReturnToMonitoring()
{
    NS_LOG_FUNCTION(this);
    if (!m_model)
    {
        return;
    }
    // A later allocation is still active. Let its own timer do the return.
    if (Simulator::Now() + NanoSeconds(1) < m_activeUntil)
    {
        return;
    }
    m_model->ChangeState(NR_UE_PDCCH_ONLY);
}

} // namespace ns3
