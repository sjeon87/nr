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

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
constexpr int64_t NR_SYMBOLS_PER_SLOT = 14; //!< OFDM symbols per NR slot
NS_LOG_COMPONENT_DEFINE("NrUePhyEnergyListener");
NS_OBJECT_ENSURE_REGISTERED(NrUePhyEnergyListener);

TypeId
NrUePhyEnergyListener::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrUePhyEnergyListener")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrUePhyEnergyListener>();
    return tid;
}

NrUePhyEnergyListener::NrUePhyEnergyListener()
    : m_slotDuration(MilliSeconds(1)),
      m_lastBwpMhz(0)
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
    // Scale active-reception power by the spatial layers actually in use.
    // TR 38.840 Section 8.1.3 antenna scaling is over Rx branches; rank <= Rx
    // branches, so this refines the existing antenna knob (active-layer scaling).
    if (rank >= 1)
    {
        m_model->ApplyAntennaScaling(rank);
    }
    m_model->ChangeState(NR_UE_PDCCH_PDSCH);
    if (m_drx)
    {
        m_drx->NotifyDataActivity();
    }
    // Hold full DL reception only for the allocated symbols, then fall back
    // to PDCCH-only monitoring for the rest of the slot.
    Time active = (numSym > 0) ? m_slotDuration * static_cast<int64_t>(numSym) / NR_SYMBOLS_PER_SLOT
                               : m_slotDuration;
    Time end = Simulator::Now() + active;
    m_activeUntil = std::max(m_activeUntil, end);
    Simulator::Schedule(active, &NrUePhyEnergyListener::ReturnToMonitoring, this);
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
    // Active only for the allocated UL symbols.
    Time active = (numSym > 0) ? m_slotDuration * static_cast<int64_t>(numSym) / NR_SYMBOLS_PER_SLOT
                               : m_slotDuration;
    Time end = Simulator::Now() + active;
    m_activeUntil = std::max(m_activeUntil, end);
    Simulator::Schedule(active, &NrUePhyEnergyListener::ReturnToMonitoring, this);
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
