// Copyright (c) 2024 University of Moratuwa
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - scaling factors sa, sf, sp definitions
//   TR 38.840 V16.0.0 (2019-06): Section 8.1 - UE power state event triggers

#include "nr-phy-energy-listener.h"

#include "nr-ue-phy.h"
#include "nr-gnb-phy.h"
// #include "nr-ue-energy-model.h"   // TODO: uncomment when class is defined
// #include "nr-gnb-energy-model.h"  // TODO: uncomment when class is defined

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPhyEnergyListener");
NS_OBJECT_ENSURE_REGISTERED(NrPhyEnergyListener);

TypeId
NrPhyEnergyListener::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrPhyEnergyListener")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrPhyEnergyListener>();
    return tid;
}

NrPhyEnergyListener::NrPhyEnergyListener()
    : m_lastDlSf(0.0),
      m_lastUlSf(0.0),
      m_lastSp(1.0),    // Default: full power (sp=1)
      m_lastSa(1.0),    // Default: all antennas active (sa=1)
      m_totalBwpRbs(0),
      m_referenceTxPowerDbm(55.0)  // TR 38.864 Set 1 reference: 55 dBm total; per-antenna varies
{
    NS_LOG_FUNCTION(this);
}

NrPhyEnergyListener::~NrPhyEnergyListener()
{
    NS_LOG_FUNCTION(this);
}

void
NrPhyEnergyListener::SetUePhy(Ptr<NrUePhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    NS_ASSERT_MSG(phy, "NrUePhy pointer must not be null");
    m_uePhy = phy;
    // TODO Week 6: connect trace sources once added to NrUePhy:
    // phy->TraceConnectWithoutContext("SlotIndication",
    //     MakeCallback(&NrPhyEnergyListener::UeSlotIndicationCallback, this));
    // phy->TraceConnectWithoutContext("PhyRxCtrlEndOk",
    //     MakeCallback(&NrPhyEnergyListener::UePdcchDecodeSuccessCallback, this));
    // phy->TraceConnectWithoutContext("PhyTxEnd",
    //     MakeCallback(&NrPhyEnergyListener::UeUlTxEndCallback, this));
}

void
NrPhyEnergyListener::SetGnbPhy(Ptr<NrGnbPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    NS_ASSERT_MSG(phy, "NrGnbPhy pointer must not be null");
    m_gnbPhy = phy;
    m_totalBwpRbs = phy->GetRbNum();
    m_referenceTxPowerDbm = phy->GetTxPower();
    // TODO Week 7: connect trace sources once added to NrGnbPhy:
    // phy->TraceConnectWithoutContext("SlotIndication",
    //     MakeCallback(&NrPhyEnergyListener::GnbSlotIndicationCallback, this));
    // phy->TraceConnectWithoutContext("PacketBurstSent",
    //     MakeCallback(&NrPhyEnergyListener::GnbDlBurstSentCallback, this));
}

void
NrPhyEnergyListener::SetUeEnergyModel(Ptr<NrUeEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_ueModel = model;
}

void
NrPhyEnergyListener::SetGnbEnergyModel(Ptr<NrGnbEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_gnbModel = model;
}

void
NrPhyEnergyListener::GnbSlotIndicationCallback(const SfnSf& sfnSf)
{
    // TODO Week 7 (revisit here in Week 3 to stub):
    //
    // Key computation per TR 38.864 §5.1:
    //
    // For each symbol (0..13) in this slot:
    //   1. Classify symbol type: DL / UL / Guard
    //      using NrGnbPhy::GetTddPattern() and slot position
    //
    //   2. Compute sa = activeTRxRUs / totalTRxRUs
    //      (from m_gnbPhy->GetActiveTxRus() if available, else 1.0)
    //
    //   3. Compute sf = allocatedDlRBs / m_totalBwpRbs
    //      (updated by GnbDlBurstSentCallback, use m_lastDlSf)
    //
    //   4. Compute sp = pow(10.0, m_gnbPhy->GetTxPower()/10.0)
    //                   / pow(10.0, m_referenceTxPowerDbm/10.0)
    //      Source: TR 38.864 §5.1 — sp = current PSD / reference PSD
    //
    //   5. Call m_gnbModel->UpdateSymbolPower(sa, sf, sp, symbolType)
    //      for each symbol
    //
    // After all 14 symbols processed:
    //   6. Call m_gnbModel->FinalizeSlotEnergy()
    //
    NS_LOG_FUNCTION(this << sfnSf);
}
void
NrPhyEnergyListener::GnbDlBurstSentCallback(uint32_t allocatedRbs)
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
NrPhyEnergyListener::GnbUlReceiveCallback(uint32_t allocatedRbs)
{
    // TODO Week 7:
    // m_lastUlSf = static_cast<double>(allocatedRbs) / m_totalBwpRbs;
    NS_LOG_FUNCTION(this << allocatedRbs);
}
void
NrPhyEnergyListener::UeSlotIndicationCallback(const SfnSf& sfnSf)
{
    // TODO Week 6 (stub here in Week 3):
    //
    // Query DRX state from NrUeMac (attached via NrUePhy pointer):
    //   If DRX_ON:  m_ueModel->ChangeState(NR_UE_PDCCH_ONLY)
    //   If DRX_OFF (long cycle): m_ueModel->ChangeState(NR_UE_LIGHT_SLEEP)
    //               or DEEP_SLEEP based on remaining inactive period vs. T_deep
    //
    NS_LOG_FUNCTION(this << sfnSf);
}

void
NrPhyEnergyListener::UePdcchDecodeSuccessCallback(bool hasDlGrant, bool hasUlGrant)
{
    // TODO Week 6:
    //
    // State transition per TR 38.840 §8.1 semantics:
    //   if (hasDlGrant):
    //     m_ueModel->ChangeState(NR_UE_PDCCH_PDSCH)  // 300 power-units FR1
    //   elif (hasUlGrant):
    //     // UL starts after UL grant delay; state transitions in UeUlTxStartCallback
    //     // For now remain in PDCCH_ONLY
    //   else:
    //     m_ueModel->ChangeState(NR_UE_PDCCH_ONLY)   // 100 power-units FR1
    NS_LOG_FUNCTION(this << hasDlGrant << hasUlGrant);
}

void
NrPhyEnergyListener::UeUlTxStartCallback(double txPowerDbm)
{
    // TODO Week 6:
    //
    // Set UE model state to UL_TX.
    // The power level depends on Tx power:
    //   - txPowerDbm <= 0  → use m_ueModel->GetUlPower0dBm()  (250 units FR1)
    //   - txPowerDbm > 0   → interpolate linearly or clamp at 23 dBm (700 units FR1)
    //     Source: TR 38.840 Table 18.
    //
    // m_ueModel->SetUlTxPowerDbm(txPowerDbm);
    // m_ueModel->ChangeState(NR_UE_UL_TX);
    NS_LOG_FUNCTION(this << txPowerDbm);
}

void
NrPhyEnergyListener::UeUlTxEndCallback()
{
    // TODO Week 6:
    //   m_ueModel->ChangeState(NR_UE_MICRO_SLEEP)  // brief micro-sleep after UL burst
    NS_LOG_FUNCTION(this);
}



double NrPhyEnergyListener::GetLastDlSf() const { return m_lastDlSf; }
double NrPhyEnergyListener::GetLastUlSf() const { return m_lastUlSf; }
double NrPhyEnergyListener::GetLastSp()   const { return m_lastSp; }
double NrPhyEnergyListener::GetLastSa()   const { return m_lastSa; }

} // namespace ns3