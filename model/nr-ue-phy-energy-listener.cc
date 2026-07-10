// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8.1 - UE power state event triggers

#include "nr-ue-phy-energy-listener.h"
#include "nr-ue-phy.h"
#include "nr-ue-energy-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

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
    // TODO Week 6: connect trace sources once added to NrUePhy:
    // phy->TraceConnectWithoutContext("SlotIndication",
    //     MakeCallback(&NrUePhyEnergyListener::SlotIndicationCallback, this));
    // phy->TraceConnectWithoutContext("PhyRxCtrlEndOk",
    //     MakeCallback(&NrUePhyEnergyListener::PdcchDecodeSuccessCallback, this));
    // phy->TraceConnectWithoutContext("PhyTxEnd",
    //     MakeCallback(&NrUePhyEnergyListener::UlTxEndCallback, this));
}

void
NrUePhyEnergyListener::SetEnergyModel(Ptr<NrUeEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    NS_ASSERT_MSG(model, "NrUeEnergyModel pointer must not be null");
    m_model = model;
}

void
NrUePhyEnergyListener::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_phy = nullptr;
    m_model = nullptr;
    Object::DoDispose();
}

void
NrUePhyEnergyListener::SlotIndicationCallback(const SfnSf& sfnSf)
{
    // TODO Week 6 (stub here in Week 3):
    //
    // Query DRX state:
    //   If DRX_ON:  m_model->ChangeState(NR_UE_PDCCH_ONLY)
    //   If DRX_OFF (long cycle): m_model->ChangeState(NR_UE_LIGHT_SLEEP)
    //               or DEEP_SLEEP based on remaining inactive period vs. T_deep
    //
    NS_LOG_FUNCTION(this << sfnSf);
    if (!m_model)
    {
        return;
    }
}

void
NrUePhyEnergyListener::PdcchDecodeSuccessCallback(bool hasDlGrant, bool hasUlGrant)
{
    // TODO Week 6:
    //
    // State transition per TR 38.840 §8.1 semantics:
    //   if (hasDlGrant):
    //     m_model->ChangeState(NR_UE_PDCCH_PDSCH)  // 300 power-units FR1
    //   elif (hasUlGrant):
    //     // UL starts after UL grant delay; state transitions in UlTxStartCallback
    //     // For now remain in PDCCH_ONLY
    //   else:
    //     m_model->ChangeState(NR_UE_PDCCH_ONLY)   // 100 power-units FR1
    NS_LOG_FUNCTION(this << hasDlGrant << hasUlGrant);
    if (!m_model)
    {
        return;
    }
}

void
NrUePhyEnergyListener::UlTxStartCallback(double txPowerDbm)
{
    // TODO Week 6:
    //
    // Set UE model state to UL_TX.
    // The power level depends on Tx power:
    //   - txPowerDbm <= 0  → use m_model->GetUlPower0dBm()  (250 units FR1)
    //   - txPowerDbm > 0   → interpolate linearly or clamp at 23 dBm (700 units FR1)
    //     Source: TR 38.840 Table 18.
    //
    // m_model->SetUlTxPowerDbm(txPowerDbm);
    // m_model->ChangeState(NR_UE_UL_TX);
    NS_LOG_FUNCTION(this << txPowerDbm);
    if (!m_model)
    {
        return;
    }
}

void
NrUePhyEnergyListener::UlTxEndCallback()
{
    // TODO Week 6:
    //   m_model->ChangeState(NR_UE_MICRO_SLEEP)  // brief micro-sleep after UL burst
    NS_LOG_FUNCTION(this);
    if (!m_model)
    {
        return;
    }
}

} // namespace ns3
