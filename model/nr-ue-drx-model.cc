// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - C-DRX power saving evaluation

#include "nr-ue-drx-model.h"

#include "nr-ue-energy-model.h"

#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrUeDrxModel");
NS_OBJECT_ENSURE_REGISTERED(NrUeDrxModel);

TypeId
NrUeDrxModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUeDrxModel")
            .SetParent<Object>()
            .SetGroupName("Nr")
            .AddConstructor<NrUeDrxModel>()
            .AddAttribute("LongCycle",
                          "DRX long cycle period (TS 38.321 drx-LongCycle).",
                          TimeValue(MilliSeconds(160)),
                          MakeTimeAccessor(&NrUeDrxModel::m_longCycle),
                          MakeTimeChecker())
            .AddAttribute("OnDuration",
                          "onDuration monitoring window (TS 38.321 drx-onDurationTimer).",
                          TimeValue(MilliSeconds(8)),
                          MakeTimeAccessor(&NrUeDrxModel::m_onDuration),
                          MakeTimeChecker())
            .AddAttribute("InactivityTimer",
                          "drx-InactivityTimer restarted on each data activity.",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrUeDrxModel::m_inactivityTimer),
                          MakeTimeChecker())
            .AddAttribute("DeepSleepThreshold",
                          "Gap to next onDuration at/above which deep sleep is used.",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&NrUeDrxModel::m_deepSleepThreshold),
                          MakeTimeChecker());
    return tid;
}

NrUeDrxModel::NrUeDrxModel()
    : m_energyModel(nullptr),
      m_longCycle(MilliSeconds(160)),
      m_onDuration(MilliSeconds(8)),
      m_inactivityTimer(MilliSeconds(10)),
      m_deepSleepThreshold(MilliSeconds(20)),
      m_nextCycleTime(Seconds(0)),
      m_inactivityRunning(false)
{
    NS_LOG_FUNCTION(this);
}

NrUeDrxModel::~NrUeDrxModel()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeDrxModel::SetEnergyModel(Ptr<NrUeEnergyModel> model)
{
    NS_LOG_FUNCTION(this << model);
    NS_ASSERT_MSG(model, "NrUeEnergyModel pointer must not be null");
    m_energyModel = model;
}

void
NrUeDrxModel::Start(Time startAt)
{
    NS_LOG_FUNCTION(this << startAt);
    m_cycleEvent = Simulator::Schedule(startAt - Simulator::Now(), &NrUeDrxModel::StartCycle, this);
}

void
NrUeDrxModel::StartCycle()
{
    NS_LOG_FUNCTION(this);
    // Wake for the onDuration: monitor PDCCH (TR 38.840 "PDCCH-only").
    if (m_energyModel)
    {
        m_energyModel->ChangeState(NR_UE_PDCCH_ONLY);
    }
    m_inactivityRunning = false;
    m_nextCycleTime = Simulator::Now() + m_longCycle;
    m_onDurationEvent = Simulator::Schedule(m_onDuration, &NrUeDrxModel::EndOnDuration, this);
    m_cycleEvent = Simulator::Schedule(m_longCycle, &NrUeDrxModel::StartCycle, this);
}

void
NrUeDrxModel::EndOnDuration()
{
    NS_LOG_FUNCTION(this);
    // If no activity extended the awake window, sleep now.
    if (!m_inactivityRunning)
    {
        GoToSleep();
    }
}

void
NrUeDrxModel::NotifyDataActivity()
{
    NS_LOG_FUNCTION(this);
    // (Re)start the inactivity timer; this keeps the UE awake past onDuration.
    m_inactivityRunning = true;
    m_inactivityEvent.Cancel();
    m_inactivityEvent =
        Simulator::Schedule(m_inactivityTimer, &NrUeDrxModel::InactivityExpired, this);
}

void
NrUeDrxModel::InactivityExpired()
{
    NS_LOG_FUNCTION(this);
    m_inactivityRunning = false;
    GoToSleep();
}

void
NrUeDrxModel::GoToSleep()
{
    NS_LOG_FUNCTION(this);
    if (!m_energyModel)
    {
        return;
    }
    // Deeper sleep only pays off over a long inactive gap (TR 38.840 Section 8.1).
    Time gap = m_nextCycleTime - Simulator::Now();
    if (gap >= m_deepSleepThreshold)
    {
        m_energyModel->ChangeState(NR_UE_DEEP_SLEEP);
    }
    else
    {
        m_energyModel->ChangeState(NR_UE_LIGHT_SLEEP);
    }
}

void
NrUeDrxModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_cycleEvent.Cancel();
    m_onDurationEvent.Cancel();
    m_inactivityEvent.Cancel();
    m_energyModel = nullptr;
    Object::DoDispose();
}

} // namespace ns3
