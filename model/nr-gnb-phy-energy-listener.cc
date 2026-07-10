// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - scaling factors sa, sf, sp definitions

#include "nr-gnb-phy-energy-listener.h"
#include "nr-gnb-phy.h"
#include "nr-gnb-energy-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

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
      m_lastUlSf(0.0),
      m_lastSp(1.0),    // Default: full power (sp=1)
      m_lastSa(1.0),    // Default: all antennas active (sa=1)
      m_totalBwpRbs(0),
      m_referenceTxPowerDbm(55.0)  // TR 38.864 Set 1 reference: 55 dBm total; per-antenna varies
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
    // TODO Week 7: connect trace sources once added to NrGnbPhy:
    // phy->TraceConnectWithoutContext("SlotIndication",
    //     MakeCallback(&NrGnbPhyEnergyListener::SlotIndicationCallback, this));
    // phy->TraceConnectWithoutContext("PacketBurstSent",
    //     MakeCallback(&NrGnbPhyEnergyListener::DlBurstSentCallback, this));
}

void
NrGnbPhyEnergyListener::SetEnergyModel(Ptr<NrGnbEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    NS_ASSERT_MSG(model, "NrGnbEnergyModel pointer must not be null");
    m_model = model;
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
NrGnbPhyEnergyListener::SlotIndicationCallback(const SfnSf& sfnSf)
{
    NS_LOG_FUNCTION(this << sfnSf);
    if (!m_model)
    {
        return;
    }
    // TODO Week 7 (revisit here in Week 3 to stub):
    //
    // Key computation per TR 38.864 §5.1:
    //
    // For each symbol (0..13) in this slot:
    //   1. Classify symbol type: DL / UL / Guard from the TDD pattern.
    //      NrGnbPhy currently exposes only SetTddPattern() (no getter), so
    //      this needs a getter added, or the pattern cached at attach time.
    //      Decision (caching vs. per-slot query) to be made before Week 7.
    //
    //   2. sa = activeTRxRUs / totalTRxRUs. No source in PHY yet, so use
    //      m_lastSa (fixed at 1.0) until antenna muting is added.
    //
    //   3. Compute sf = allocatedDlRBs / m_totalBwpRbs
    //      (updated by DlBurstSentCallback, use m_lastDlSf)
    //
    //   4. RefreshSp() to update m_lastSp from current Tx power.
    //
    //   5. Call m_model->UpdateSymbolPower(sa, sf, sp, symbolType)
    //      for each symbol
    //
    // After all 14 symbols processed:
    //   6. Call m_model->FinalizeSlotEnergy()
}

void
NrGnbPhyEnergyListener::DlBurstSentCallback(uint32_t allocatedRbs)
{
    // TODO Week 7:
    //
    // sf = allocatedRbs / m_totalBwpRbs
    // Source: TR 38.864 §5.1 — sf = fraction of BW carrying DL data
    //
    // Guard: if m_totalBwpRbs == 0, log a warning and return
    // m_lastDlSf = static_cast<double>(allocatedRbs) / m_totalBwpRbs;
    NS_LOG_FUNCTION(this << allocatedRbs);
}

void
NrGnbPhyEnergyListener::UlReceiveCallback(uint32_t allocatedRbs)
{
    // TODO Week 7:
    // m_lastUlSf = static_cast<double>(allocatedRbs) / m_totalBwpRbs;
    NS_LOG_FUNCTION(this << allocatedRbs);
}

void
NrGnbPhyEnergyListener::RefreshSp()
{
    NS_LOG_FUNCTION(this);
    if (!m_phy)
    {
        return;
    }
    // sp = current Tx power / reference Tx power (linear).
    // m_referenceTxPowerDbm is captured from m_phy->GetTxPower() at attach
    // time in SetPhy(); the current value below is re-read each slot so
    // runtime Tx-power changes (e.g. power control) are reflected.
    double curLin = std::pow(10.0, m_phy->GetTxPower() / 10.0);
    double refLin = std::pow(10.0, m_referenceTxPowerDbm / 10.0);
    m_lastSp = (refLin > 0.0) ? (curLin / refLin) : 1.0;
}

double NrGnbPhyEnergyListener::GetLastDlSf() const { return m_lastDlSf; }
double NrGnbPhyEnergyListener::GetLastUlSf() const { return m_lastUlSf; }
double NrGnbPhyEnergyListener::GetLastSp()   const { return m_lastSp; }
double NrGnbPhyEnergyListener::GetLastSa()   const { return m_lastSa; }

} // namespace ns3
