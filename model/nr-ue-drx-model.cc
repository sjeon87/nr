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

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

namespace ns3
{
struct DrxRefConfig
{
    Time cycle;
    Time onDurationFr1;
    Time onDurationFr2;
    Time inactivityTimer;
};

// TR 38.840 Section 8.2 traffic-model reference configurations, indexed to
// NrUeDrxModel::ReferenceConfig - 1 (CUSTOM has no entry).
const DrxRefConfig DRX_REF_CONFIG[] = {
    {MilliSeconds(160), MilliSeconds(8), MilliSeconds(4), MilliSeconds(100)},  // FTP_160MS
    {MilliSeconds(320), MilliSeconds(10), MilliSeconds(5), MilliSeconds(80)},  // INSTANT_MSG_320MS
    {MilliSeconds(40), MilliSeconds(4), MilliSeconds(2), MilliSeconds(10)},    // VOIP_40MS
};
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
                          "Gap to next onDuration at/above which deep sleep is used "
                          "(TR 38.840 Table 19 deep-sleep total transition time = 20 ms).",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&NrUeDrxModel::m_deepSleepThreshold),
                          MakeTimeChecker())
            .AddAttribute("LightSleepThreshold",
                          "Gap to next onDuration at/above which light sleep is used. Below "
                          "this the gap cannot cover light sleep's own transition cost, so "
                          "micro sleep (zero transition, TR 38.840 Table 19) is used instead.",
                          TimeValue(MilliSeconds(6)),
                          MakeTimeAccessor(&NrUeDrxModel::m_lightSleepThreshold),
                          MakeTimeChecker())
            .AddAttribute("FreqRange",
                          "Frequency range, used to pick the FR-appropriate onDuration "
                          "when ReferenceConfig is not CUSTOM (TR 38.840 Section 8.2).",
                          EnumValue(FR1),
                          MakeEnumAccessor<FreqRange>(&NrUeDrxModel::m_freqRange),
                          MakeEnumChecker(FR1, "FR1", FR2, "FR2"))
            .AddAttribute("ReferenceConfig",
                          "Named TR 38.840 Section 8.2 reference DRX configuration. "
                          "Non-CUSTOM values override LongCycle/OnDuration/InactivityTimer "
                          "with one coherent, spec-cited combination at Start().",
                          EnumValue(CUSTOM),
                          MakeEnumAccessor<ReferenceConfig>(&NrUeDrxModel::m_referenceConfig),
                          MakeEnumChecker(CUSTOM,
                                          "CUSTOM",
                                          FTP_160MS,
                                          "FTP_160MS",
                                          INSTANT_MSG_320MS,
                                          "INSTANT_MSG_320MS",
                                          VOIP_40MS,
                                          "VOIP_40MS"));
    return tid;
}

NrUeDrxModel::NrUeDrxModel()
    : m_energyModel(nullptr),
      m_longCycle(MilliSeconds(160)),
      m_onDuration(MilliSeconds(8)),
      m_inactivityTimer(MilliSeconds(10)),
      m_deepSleepThreshold(MilliSeconds(20)),
      m_lightSleepThreshold(MilliSeconds(6)),
      m_nextCycleTime(Seconds(0)),
      m_inactivityRunning(false),
      m_onDurationRunning(false)
{
    NS_LOG_FUNCTION(this);
}

NrUeDrxModel::~NrUeDrxModel()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeDrxModel::ApplyReferenceConfig()
{
    NS_LOG_FUNCTION(this);
    if (m_referenceConfig == CUSTOM)
    {
        return;
    }
    const DrxRefConfig& cfg = DRX_REF_CONFIG[m_referenceConfig - 1];
    m_longCycle = cfg.cycle;
    m_onDuration = (m_freqRange == FR2) ? cfg.onDurationFr2 : cfg.onDurationFr1;
    m_inactivityTimer = cfg.inactivityTimer;
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
    ApplyReferenceConfig();
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
    m_onDurationRunning = true;
    m_nextCycleTime = Simulator::Now() + m_longCycle;
    m_onDurationEvent = Simulator::Schedule(m_onDuration, &NrUeDrxModel::EndOnDuration, this);
    m_cycleEvent = Simulator::Schedule(m_longCycle, &NrUeDrxModel::StartCycle, this);
}

void
NrUeDrxModel::EndOnDuration()
{
    NS_LOG_FUNCTION(this);
    m_onDurationRunning = false;
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
    if (!m_onDurationRunning)
    {
        GoToSleep();
    }
}

void
NrUeDrxModel::GoToSleep()
{
    NS_LOG_FUNCTION(this);
    if (!m_energyModel)
    {
        return;
    }
    // TR 38.840: a sleep interval must exceed that state's total transition time
    // (Table 19), otherwise the transition cost outweighs the saving.
    Time gap = m_nextCycleTime - Simulator::Now();
    if (gap >= m_deepSleepThreshold)
    {
        m_energyModel->ChangeState(NR_UE_DEEP_SLEEP);
    }
    else if (gap >= m_lightSleepThreshold)
    {
        m_energyModel->ChangeState(NR_UE_LIGHT_SLEEP);
    }
    else
    {
        // Too short even for light sleep's 6 ms transition. Micro sleep has zero
        // transition time so it is always valid regardless of gap length.
        m_energyModel->ChangeState(NR_UE_MICRO_SLEEP);
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
