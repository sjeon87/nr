// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - scaling factors sa, sf, sp definitions
//   TR 38.864 V18.1.0 (2023-03): Section 5.2 - symbol-level time-domain energy

#include "nr-gnb-phy-energy-listener.h"

#include "nr-gnb-energy-model.h"
#include "nr-gnb-phy.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <string>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbPhyEnergyListener");
NS_OBJECT_ENSURE_REGISTERED(NrGnbPhyEnergyListener);

TypeId
NrGnbPhyEnergyListener::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrGnbPhyEnergyListener")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrGnbPhyEnergyListener>();
    return tid;
}

NrGnbPhyEnergyListener::NrGnbPhyEnergyListener()
    : m_lastDlSf(0.0),
      m_lastSp(1.0), // Default: full power (sp=1)
      m_lastSa(1.0), // Default: all antennas active (sa=1)
      m_totalBwpRbs(0),
      m_dlCapable(true),
      m_symbolsPerSlot(14) // Pre-attach default, matching NrPhy's own
{
    NS_LOG_FUNCTION(this);
}

NrGnbPhyEnergyListener::~NrGnbPhyEnergyListener()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbPhyEnergyListener::SetPhy(Ptr<NrGnbPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    NS_ASSERT_MSG(phy, "NrGnbPhy pointer must not be null");
    m_phy = phy;
    m_totalBwpRbs = phy->GetRbNum();
    // NrGnbPhy builds the slot's symbol-class map from GetSymbolsPerSlot(), so the
    // idle count below must be derived against the same value (12 or 14).
    m_symbolsPerSlot = phy->GetSymbolsPerSlot();
    // A bandwidth part whose pattern contains no downlink slot is uplink-only, so
    // it sits outside the carrier's DL reference bandwidth and must not widen the
    // sf denominator. Only LteNrTddSlotType::UL carries no downlink - DL, S and F
    // all include DL control and DL data.
    const std::string pattern = phy->GetPattern();
    m_dlCapable = pattern.find("DL") != std::string::npos ||
                  pattern.find('F') != std::string::npos || pattern.find('S') != std::string::npos;
    if (m_model)
    {
        m_model->SetSymbolDuration(phy->GetSymbolPeriod());
    }
    phy->TraceConnectWithoutContext(
        "SlotEnergyStats",
        MakeCallback(&NrGnbPhyEnergyListener::SlotEnergyStatsCallback, this));
}

void
NrGnbPhyEnergyListener::SetEnergyModel(Ptr<NrGnbEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    NS_ASSERT_MSG(model, "NrGnbEnergyModel pointer must not be null");

    m_model = model;
    if (m_phy)
    {
        m_model->SetSymbolDuration(m_phy->GetSymbolPeriod());
    }
}

void
NrGnbPhyEnergyListener::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_phy = nullptr;
    m_model = nullptr;
    Object::DoDispose();
}

void
NrGnbPhyEnergyListener::SlotEnergyStatsCallback(const SfnSf& sfnSf,
                                                uint32_t availableRb,
                                                uint16_t dlDataMask,
                                                uint32_t dlDataReg,
                                                uint16_t ulMask,
                                                uint16_t dlCtrlMask,
                                                uint32_t dlCtrlReg,
                                                uint16_t bwpId,
                                                uint16_t cellId)
{
    NS_LOG_FUNCTION(this << sfnSf << availableRb << dlDataMask << ulMask << dlCtrlMask << bwpId
                         << cellId);
    if (!m_model)
    {
        return;
    }
    RefreshSp();

    // Report what this BWP did and let the model apply TR 38.864: sf is a property
    // of the whole carrier, which one BWP cannot see. Several BWPs of one carrier
    // can drive the same model this way, and their occupancies are combined before
    // the power formula is applied, so the carrier's static baseline P3 and its
    // load-independent share A are counted once rather than once per BWP
    // (Section 5.1: scaling on "occupied BW/RBs [...] in one CC").
    NrGnbEnergyModel::BwpOccupancy occ;
    occ.rbCount = m_totalBwpRbs;
    occ.dlDataMask = dlDataMask;
    occ.dlDataReg = dlDataReg;
    occ.dlCtrlMask = dlCtrlMask;
    occ.dlCtrlReg = dlCtrlReg;
    occ.ulMask = ulMask;
    occ.dlCapable = m_dlCapable;
    occ.txPowerLin = std::pow(10.0, m_phy->GetTxPower() / 10.0);
    occ.symbolsPerSlot = m_symbolsPerSlot;
    occ.symbolDuration = m_phy->GetSymbolPeriod();
    // A BWP that stops reporting must stop contributing rather than linger with
    // its last slot's occupancy.
    occ.validUntil = Simulator::Now() + m_phy->GetSlotPeriod();

    m_lastDlSf = m_model->CalcSf(dlDataReg, std::popcount(dlDataMask), availableRb);
    m_model->ReportBwpOccupancy(bwpId, occ);
}

void
NrGnbPhyEnergyListener::RefreshSp()
{
    NS_LOG_FUNCTION(this);
    if (!m_phy || !m_model)
    {
        return;
    }
    // The energy model owns the configured 3GPP reference Tx power, so it
    // computes sp; the listener only reports the current Tx power.
    m_model->SetTxPowerDbm(m_phy->GetTxPower());
    m_lastSp = m_model->GetSp();
}

double
NrGnbPhyEnergyListener::GetLastDlSf() const
{
    return m_lastDlSf;
}

double
NrGnbPhyEnergyListener::GetLastSp() const
{
    return m_lastSp;
}

double
NrGnbPhyEnergyListener::GetLastSa() const
{
    return m_lastSa;
}

bool
NrGnbPhyEnergyListener::IsDlCapable() const
{
    return m_dlCapable;
}

} // namespace ns3
