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
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbPhyEnergyListener");
NS_OBJECT_ENSURE_REGISTERED(NrGnbPhyEnergyListener);

namespace
{
constexpr uint32_t SYMBOLS_PER_SLOT = 14; //!< OFDM symbols per NR slot (normal CP)
} // namespace

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
      m_lastUlSf(0.0),
      m_lastSp(1.0), // Default: full power (sp=1)
      m_lastSa(1.0), // Default: all antennas active (sa=1)
      m_totalBwpRbs(0),
      m_referenceTxPowerDbm(55.0) // TR 38.864 Set 1 reference; refreshed at attach
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
    m_referenceTxPowerDbm = phy->GetTxPower();
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
                                                uint32_t dlDataSym,
                                                uint32_t dlDataReg,
                                                uint32_t ulDataSym,
                                                uint32_t dlCtrlSym,
                                                uint32_t dlCtrlReg,
                                                uint32_t ulCtrlSym,
                                                uint16_t bwpId,
                                                uint16_t cellId)
{
    NS_LOG_FUNCTION(this << sfnSf << availableRb << dlDataSym << ulDataSym << dlCtrlSym << ulCtrlSym
                         << bwpId << cellId);
    if (!m_model)
    {
        return;
    }
    RefreshSp();

    // sf = used REGs / (RBs in band * used symbols): the fraction of the band
    // occupied during the active symbols (TR 38.864 Section 5.1 definition of sf).
    // Computed separately for the DL data region and the DL control (PDCCH)
    // region; the UL power formula has no sf dependence.
    auto sf = [availableRb](uint32_t reg, uint32_t sym) {
        return (availableRb > 0 && sym > 0)
                   ? std::min(1.0,
                              static_cast<double>(reg) / (static_cast<double>(availableRb) * sym))
                   : 0.0;
    };
    double sfData = sf(dlDataReg, dlDataSym);
    double sfCtrl = sf(dlCtrlReg, dlCtrlSym);
    m_lastDlSf = sfData;

    // Direction-aware per-symbol timeline (TR 38.864 Section 5.2). The gNB
    // transmits during DL data and DL control (PDCCH), and receives during UL
    // data and UL control (PUCCH/SRS), so DL symbols use P_DL(sf) and UL symbols
    // use P_UL. Symbols carrying no allocation are idle (micro-sleep, P3). sa is
    // held at 1.0 until antenna muting is modelled.
    uint32_t scheduled = dlDataSym + ulDataSym + dlCtrlSym + ulCtrlSym;
    uint32_t idleSym = (scheduled < SYMBOLS_PER_SLOT) ? (SYMBOLS_PER_SLOT - scheduled) : 0;
    for (uint32_t s = 0; s < dlDataSym; ++s)
    {
        m_model->UpdateSymbolPower(m_lastSa, sfData, m_lastSp, NrGnbSymbolType::Dl);
    }
    for (uint32_t s = 0; s < dlCtrlSym; ++s)
    {
        m_model->UpdateSymbolPower(m_lastSa, sfCtrl, m_lastSp, NrGnbSymbolType::Dl);
    }
    for (uint32_t s = 0; s < ulDataSym; ++s)
    {
        m_model->UpdateSymbolPower(m_lastSa, 0.0, m_lastSp, NrGnbSymbolType::Ul);
    }
    for (uint32_t s = 0; s < ulCtrlSym; ++s)
    {
        m_model->UpdateSymbolPower(m_lastSa, 0.0, m_lastSp, NrGnbSymbolType::Ul);
    }
    for (uint32_t s = 0; s < idleSym; ++s)
    {
        m_model->UpdateSymbolPower(m_lastSa, 0.0, m_lastSp, NrGnbSymbolType::Idle);
    }
    m_model->FinalizeSlotEnergy();
}

void
NrGnbPhyEnergyListener::RefreshSp()
{
    NS_LOG_FUNCTION(this);
    if (!m_phy)
    {
        return;
    }
    // sp = current Tx power / reference Tx power (linear). The reference is
    // captured from m_phy->GetTxPower() at attach time in SetPhy(); the
    // current value below is re-read each slot so runtime Tx-power changes
    // (e.g. power control) are reflected.
    double curLin = std::pow(10.0, m_phy->GetTxPower() / 10.0);
    double refLin = std::pow(10.0, m_referenceTxPowerDbm / 10.0);
    m_lastSp = (refLin > 0.0) ? std::min(1.0, curLin / refLin) : 1.0;
}

double
NrGnbPhyEnergyListener::GetLastDlSf() const
{
    return m_lastDlSf;
}

double
NrGnbPhyEnergyListener::GetLastUlSf() const
{
    return m_lastUlSf;
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

} // namespace ns3
