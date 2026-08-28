// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8.1 - UE power states and scaling

#include "nr-ue-energy-model.h"

#include "ns3/double.h"
#include "ns3/energy-source.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrUeEnergyModel");
NS_OBJECT_ENSURE_REGISTERED(NrUeEnergyModel);

// TR 38.840 Table 18 relative power values (Deep Sleep = 1). Stored as tables
// keyed by frequency range because a single attribute default cannot express
// the "FR1 value / FR2 value" pairs the standard defines.
namespace
{
// Index by NrUePowerState (excluding the NR_UE_NUM_STATES sentinel).
constexpr double UE_POWER_FR1[NR_UE_NUM_STATES] = {
    1.0,   // NR_UE_DEEP_SLEEP
    20.0,  // NR_UE_LIGHT_SLEEP
    45.0,  // NR_UE_MICRO_SLEEP
    100.0, // NR_UE_PDCCH_ONLY
    100.0, // NR_UE_SSB_CSI_RS
    300.0, // NR_UE_PDCCH_PDSCH
    250.0  // NR_UE_UL_TX (0 dBm anchor; 23 dBm anchor = 700, see SetUlTxPowerDbm)
};
constexpr double UE_POWER_FR2[NR_UE_NUM_STATES] = {
    1.0,   // NR_UE_DEEP_SLEEP
    20.0,  // NR_UE_LIGHT_SLEEP
    38.0,  // NR_UE_MICRO_SLEEP
    175.0, // NR_UE_PDCCH_ONLY
    175.0, // NR_UE_SSB_CSI_RS
    350.0, // NR_UE_PDCCH_PDSCH
    350.0  // NR_UE_UL_TX (single FR2 value)
};

constexpr double UE_UL_POWER_FR1_0DBM = 250.0;   // TR 38.840 Table 18
constexpr double UE_UL_POWER_FR1_23DBM = 700.0;  // TR 38.840 Table 18
constexpr double UE_BWP_TRANSITION_FLOOR = 50.0; // TR 38.840 Section 8.1.3
} // namespace

TypeId
NrUeEnergyModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUeEnergyModel")
            .SetParent<energy::DeviceEnergyModel>()
            .SetGroupName("Nr")
            .AddConstructor<NrUeEnergyModel>()
            .AddAttribute("FreqRange",
                          "Frequency range selecting the TR 38.840 power table.",
                          EnumValue(FR1),
                          MakeEnumAccessor<FreqRange>(&NrUeEnergyModel::m_freqRange),
                          MakeEnumChecker(FR1, "FR1", FR2, "FR2"))
            .AddAttribute("PowerUnit",
                          "Absolute power of one TR 38.840 relative power-unit, in mW.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrUeEnergyModel::m_powerUnitMw),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ReferenceRxAntennas",
                          "Reference number of receive antennas (TR 38.840 Section 8.1.3).",
                          UintegerValue(4),
                          MakeUintegerAccessor(&NrUeEnergyModel::m_refRxAntennas),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("ReferenceBwpBandwidth",
                          "Reference BWP bandwidth in MHz (TR 38.840 Section 8.1.3).",
                          UintegerValue(100),
                          MakeUintegerAccessor(&NrUeEnergyModel::m_refBwpMhz),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("SetupTransitionPower",
                          "Transient extra power [W] for a sleep->active (RRC setup) transition. "
                          "0 = disabled (instantaneous, free transitions).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrUeEnergyModel::m_setupTransitionPowerW),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SetupTransitionTime",
                          "Duration of the setup/RRC transition transient.",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&NrUeEnergyModel::m_setupTransitionTime),
                          MakeTimeChecker())
            .AddTraceSource("State",
                            "Current UE power state (NrUePowerState as int).",
                            MakeTraceSourceAccessor(&NrUeEnergyModel::m_stateTrace),
                            "ns3::TracedValueCallback::Int32")
            .AddTraceSource("InstantaneousPower",
                            "Instantaneous UE power draw [W] at each state change.",
                            MakeTraceSourceAccessor(&NrUeEnergyModel::m_powerTrace),
                            "ns3::TracedValueCallback::Double")
            .AddTraceSource("TotalEnergyConsumption",
                            "Total energy consumed by the UE [J].",
                            MakeTraceSourceAccessor(&NrUeEnergyModel::m_totalEnergyJ),
                            "ns3::TracedValueCallback::Double");
    return tid;
}

NrUeEnergyModel::NrUeEnergyModel()
    : m_source(nullptr),
      m_freqRange(FR1),
      m_powerUnitMw(1.0),
      m_refRxAntennas(4),
      m_refBwpMhz(100),
      m_currentState(NR_UE_DEEP_SLEEP),
      m_lastUpdateTime(Seconds(0)),
      m_ulRelativePower(UE_UL_POWER_FR1_0DBM),
      m_stateTimeS{},
      m_occupancyStart(Seconds(0)),
      m_bwpScale(1.0),
      m_antennaScale(1.0),
      m_bdScale(1.0),
      m_transitionExtraW(0.0),
      m_transitionEndTime(Seconds(0)),
      m_setupTransitionPowerW(0.0),
      m_setupTransitionTime(MilliSeconds(20)),
      m_stateTrace(NR_UE_DEEP_SLEEP),
      m_powerTrace(0.0),
      m_totalEnergyJ(0.0)
{
    NS_LOG_FUNCTION(this);
}

NrUeEnergyModel::~NrUeEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

double
NrUeEnergyModel::GetRelativePower(NrUePowerState state) const
{
    NS_LOG_FUNCTION(this << state);
    NS_ASSERT_MSG(state < NR_UE_NUM_STATES, "Invalid UE power state");
    if (state == NR_UE_UL_TX)
    {
        // UL power is level-dependent (set via SetUlTxPowerDbm); use the cached
        // value rather than the table anchor so power control is reflected.
        return m_ulRelativePower;
    }
    return (m_freqRange == FR1) ? UE_POWER_FR1[state] : UE_POWER_FR2[state];
}

bool
NrUeEnergyModel::IsActiveDlState(NrUePowerState state)
{
    return state == NR_UE_PDCCH_ONLY || state == NR_UE_SSB_CSI_RS || state == NR_UE_PDCCH_PDSCH;
}

double
NrUeEnergyModel::GetStatePowerW(NrUePowerState state) const
{
    NS_LOG_FUNCTION(this << state);
    double relative = GetRelativePower(state);
    if (IsActiveDlState(state))
    {
        relative *= m_bwpScale * m_antennaScale * m_bdScale;
    }
    return relative * m_powerUnitMw * 1e-3;
}

double
NrUeEnergyModel::ScaleBwp(uint32_t bandwidthMhz) const
{
    NS_LOG_FUNCTION(this << bandwidthMhz);
    // TR 38.840 Section 8.1.3: scale(X) = 0.4 + 0.6 * (X - 20) / 80.
    double scale = 0.4 + 0.6 * (static_cast<double>(bandwidthMhz) - 20.0) / 80.0;
    // The scaled active power must not fall below the BWP transition floor.
    // Express the floor relative to the reference PDCCH+PDSCH power so the
    // factor stays in (0, 1].
    double floorFactor = UE_BWP_TRANSITION_FLOOR / UE_POWER_FR1[NR_UE_PDCCH_PDSCH];
    return std::clamp(scale, floorFactor, 1.0);
}

void
NrUeEnergyModel::ApplyBwpScaling(uint32_t bandwidthMhz)
{
    NS_LOG_FUNCTION(this << bandwidthMhz);
    m_bwpScale = ScaleBwp(bandwidthMhz);
}

void
NrUeEnergyModel::ApplyAntennaScaling(uint32_t activeAntennas)
{
    NS_LOG_FUNCTION(this << activeAntennas);
    NS_ASSERT_MSG(activeAntennas >= 1, "At least one receive antenna required");
    // Each halving of the receive chains relative to the reference applies one
    // 0.7 factor (TR 38.840 Section 8.1.3: P_2Rx = 0.7 * P_4Rx, etc.).
    double factor = 1.0;
    uint32_t ant = m_refRxAntennas;
    while (ant > activeAntennas && ant > 1)
    {
        factor *= 0.7;
        ant /= 2;
    }
    m_antennaScale = factor;
}

void
NrUeEnergyModel::ApplyBdReduction(double alpha)
{
    NS_LOG_FUNCTION(this << alpha);
    NS_ASSERT_MSG(alpha > 0.0 && alpha <= 1.0, "alpha must be in (0, 1]");
    // TR 38.840 Section 8.1.3: P(alpha) = alpha*Pt + (1-alpha)*0.7*Pt.
    m_bdScale = alpha + (1.0 - alpha) * 0.7;
}

void
NrUeEnergyModel::SetUlTxPowerDbm(double txPowerDbm)
{
    NS_LOG_FUNCTION(this << txPowerDbm);
    if (m_freqRange == FR2)
    {
        m_ulRelativePower = UE_POWER_FR2[NR_UE_UL_TX];
        return;
    }
    // FR1: linearly interpolate between the 0 dBm and 23 dBm anchors.
    double frac = std::clamp(txPowerDbm / 23.0, 0.0, 1.0);
    m_ulRelativePower =
        UE_UL_POWER_FR1_0DBM + frac * (UE_UL_POWER_FR1_23DBM - UE_UL_POWER_FR1_0DBM);
}

void
NrUeEnergyModel::ChangeState(int newState)
{
    NS_LOG_FUNCTION(this << newState);
    NS_ASSERT_MSG(newState >= 0 && newState < NR_UE_NUM_STATES, "Invalid UE state");

    Time now = Simulator::Now();
    double durationS = (now - m_lastUpdateTime).GetSeconds();
    // Accrue the energy and the occupancy time of the state we are leaving.
    m_totalEnergyJ = m_totalEnergyJ + GetCurrentPowerW() * durationS;
    m_stateTimeS[m_currentState] += durationS;
    m_lastUpdateTime = now;
    m_currentState = static_cast<NrUePowerState>(newState);

    m_stateTrace = newState;
    m_powerTrace = GetCurrentPowerW();
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
}

NrUePowerState
NrUeEnergyModel::GetCurrentState() const
{
    return m_currentState;
}

double
NrUeEnergyModel::GetCurrentPowerW() const
{
    NS_LOG_FUNCTION(this);
    double w = GetStatePowerW(m_currentState);
    if (Simulator::Now() < m_transitionEndTime)
    {
        w += m_transitionExtraW; // transition transient riding on top of the state
    }
    return w;
}

void
NrUeEnergyModel::TriggerTransition(double extraPowerW, Time duration)
{
    NS_LOG_FUNCTION(this << extraPowerW << duration);
    if (extraPowerW <= 0.0 || duration <= Seconds(0))
    {
        return;
    }
    // Flush the open interval, then ride the transient for `duration`.
    Time now = Simulator::Now();
    double durationS = (now - m_lastUpdateTime).GetSeconds();
    m_totalEnergyJ = m_totalEnergyJ + GetCurrentPowerW() * durationS;
    m_stateTimeS[m_currentState] += durationS;
    m_lastUpdateTime = now;
    m_transitionExtraW = extraPowerW;
    m_transitionEndTime = now + duration;
    m_powerTrace = GetCurrentPowerW();
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
    Simulator::Schedule(duration, &NrUeEnergyModel::EndTransition, this);
}

void
NrUeEnergyModel::TriggerSetupTransition()
{
    NS_LOG_FUNCTION(this);
    TriggerTransition(m_setupTransitionPowerW, m_setupTransitionTime);
}

void
NrUeEnergyModel::EndTransition()
{
    NS_LOG_FUNCTION(this);
    // Commit the transition interval at (state + transient), then settle.
    Time now = Simulator::Now();
    double durationS = (now - m_lastUpdateTime).GetSeconds();
    double powerW = GetStatePowerW(m_currentState) + m_transitionExtraW;
    m_totalEnergyJ = m_totalEnergyJ + powerW * durationS;
    m_stateTimeS[m_currentState] += durationS;
    m_lastUpdateTime = now;
    m_transitionExtraW = 0.0;
    m_transitionEndTime = now;
    m_powerTrace = GetCurrentPowerW();
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
}

double
NrUeEnergyModel::GetTotalEnergyJ() const
{
    NS_LOG_FUNCTION(this);
    double durationS = (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    return m_totalEnergyJ + GetCurrentPowerW() * durationS;
}

double
NrUeEnergyModel::GetStateTimeFraction(NrUePowerState state) const
{
    NS_LOG_FUNCTION(this << state);
    NS_ASSERT_MSG(state < NR_UE_NUM_STATES, "Invalid UE power state");
    // Snapshot including the still-open interval in the current state.
    double perState[NR_UE_NUM_STATES];
    double total = 0.0;
    for (int s = 0; s < NR_UE_NUM_STATES; ++s)
    {
        perState[s] = m_stateTimeS[s];
    }
    perState[m_currentState] += (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    for (int s = 0; s < NR_UE_NUM_STATES; ++s)
    {
        total += perState[s];
    }
    return (total > 0.0) ? perState[state] / total : 0.0;
}

double
NrUeEnergyModel::GetAverageRelativePower() const
{
    NS_LOG_FUNCTION(this);
    double avg = 0.0;
    for (int s = 0; s < NR_UE_NUM_STATES; ++s)
    {
        avg += GetStateTimeFraction(static_cast<NrUePowerState>(s)) *
               GetRelativePower(static_cast<NrUePowerState>(s));
    }
    return avg;
}

void
NrUeEnergyModel::ResetOccupancy()
{
    NS_LOG_FUNCTION(this);
    for (int s = 0; s < NR_UE_NUM_STATES; ++s)
    {
        m_stateTimeS[s] = 0.0;
    }
    m_occupancyStart = Simulator::Now();
    m_lastUpdateTime = Simulator::Now();
}

void
NrUeEnergyModel::SetEnergySource(Ptr<energy::EnergySource> source)
{
    NS_LOG_FUNCTION(this << source);
    NS_ASSERT(source);
    m_source = source;
}

double
NrUeEnergyModel::GetTotalEnergyConsumption() const
{
    return GetTotalEnergyJ();
}

void
NrUeEnergyModel::HandleEnergyDepletion()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeEnergyModel::HandleEnergyRecharged()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeEnergyModel::HandleEnergyChanged()
{
    NS_LOG_FUNCTION(this);
}

double
NrUeEnergyModel::DoGetCurrentA() const
{
    NS_LOG_FUNCTION(this);
    double voltage = m_source ? m_source->GetSupplyVoltage() : 1.0;
    return (voltage > 0.0) ? GetCurrentPowerW() / voltage : 0.0;
}

void
NrUeEnergyModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_source = nullptr;
    energy::DeviceEnergyModel::DoDispose();
}

} // namespace ns3
