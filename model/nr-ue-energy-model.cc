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
#include <map>

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
// TR 38.840 Table 18 (FR1) relative power values
constexpr double UE_POWER_FR1[NR_UE_NUM_STATES] = {
    1.0,   // NR_UE_DEEP_SLEEP
    20.0,  // NR_UE_LIGHT_SLEEP
    45.0,  // NR_UE_MICRO_SLEEP
    100.0, // NR_UE_PDCCH_ONLY
    100.0, // NR_UE_SSB_CSI_RS
    300.0, // NR_UE_PDCCH_PDSCH
    250.0  // NR_UE_UL_TX (0 dBm anchor; 23 dBm anchor = 700, see SetUlTxPowerDbm)
};
// TR 38.840 Table 20 (FR2) relative power value
constexpr double UE_POWER_FR2[NR_UE_NUM_STATES] = {
    1.0,   // NR_UE_DEEP_SLEEP
    20.0,  // NR_UE_LIGHT_SLEEP
    45.0,  // NR_UE_MICRO_SLEEP
    175.0, // NR_UE_PDCCH_ONLY
    175.0, // NR_UE_SSB_CSI_RS
    350.0, // NR_UE_PDCCH_PDSCH
    350.0  // NR_UE_UL_TX (single FR2 value)
};

constexpr double UE_UL_POWER_FR1_0DBM = 250.0;   // TR 38.840 Table 18
constexpr double UE_UL_POWER_FR1_23DBM = 700.0;  // TR 38.840 Table 18
constexpr double UE_BWP_TRANSITION_FLOOR = 50.0; // TR 38.840 Section 8.1.3

// TR 38.840 Table 19: additional transition energy [relative power x ms] and
// total transition time, keyed by the state being entered. Only states that HAVE
// a transition appear: a lookup miss means zero, so the "only sleep costs
// anything" rule is structural rather than five rows of padding. Micro sleep is
// absent because it is entered and left immediately.
// (Time is not constexpr-constructible, so this is a runtime-const map.)
struct SleepTransition
{
    double energyRelMs; //!< additional transition energy [relative power x ms]
    Time totalTime;     //!< total transition time
};

const std::map<NrUePowerState, SleepTransition> UE_TRANSITIONS{
    {NR_UE_DEEP_SLEEP, {450.0, MilliSeconds(20)}},
    {NR_UE_LIGHT_SLEEP, {100.0, MilliSeconds(6)}},
};
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
                          "Absolute power of one TR 38.840 relative power-unit, in mW. "
                          "PowerUnit is a user-supplied calibration assumption. 3GPP "
                          "provides only relative power, and does not provide the default "
                          "value for the absolute value of the unit. Hence the result "
                          "obtained with this default value do not represent the absolute "
                          "energy consumption per 3GPP model.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrUeEnergyModel::m_powerUnitMw),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ReferenceRxAntennas",
                          "Reference receive-chain count (TR 38.840 Table 21). 0 = derive "
                          "from FreqRange (4 for FR1, 2 for FR2).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrUeEnergyModel::m_refRxAntennas),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ActiveRxChains",
                          "Powered UE receive chains for the TR 38.840 Table 21 antenna "
                          "scaling (one 0.7 factor per halving relative to "
                          "ReferenceRxAntennas). 0 = equal to ReferenceRxAntennas, i.e. no "
                          "scaling. 5G-LENA does not model receive-chain adaptation, so this "
                          "is a static configuration, not a simulated quantity.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrUeEnergyModel::m_activeRxChains),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ReferenceBwpBandwidth",
                          "Reference BWP bandwidth in MHz (TR 38.840 Section 8.1.3).",
                          UintegerValue(100),
                          MakeUintegerAccessor(&NrUeEnergyModel::m_refBwpMhz),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("SetupTransitionPower",
                          "OPTIONAL empirical override: extra transient power [W] on a "
                          "sleep->active (RRC setup) transition. NOT a 3GPP value - the "
                          "TR 38.840 Table 19 transition energy is applied automatically on "
                          "sleep entry (see GetTransitionEnergyJ). 0 = disabled.",
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
                            "Instantaneous UE power draw [W]. Fires on every change of the "
                            "drawn power: state changes, scaling changes and the start and "
                            "end of a transition transient.",
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
      m_activeRxChains(0),
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

void
NrUeEnergyModel::DoInitialize()
{
    m_refRxAntennas = GetRefRxAntennas();
    // Chain adaptation is not modelled, so unset means all reference chains on.
    if (m_activeRxChains == 0)
    {
        m_activeRxChains = m_refRxAntennas;
    }
    ApplyAntennaScaling(m_activeRxChains);
    energy::DeviceEnergyModel::DoInitialize();
}

uint32_t
NrUeEnergyModel::GetRefRxAntennas() const
{
    // The attribute default is a 0 sentinel meaning "derive from FreqRange".
    // Resolve it on demand so the scaling is correct even before DoInitialize().
    return (m_refRxAntennas != 0) ? m_refRxAntennas
                                  : ((m_freqRange == FR2) ? 2 : 4); // TR 38.840 Table 21
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
    if (m_freqRange == FR2)
    {
        NS_LOG_WARN("ScaleBwp: the 0.4 + 0.6*(X-20)/80 anchors are the TR 38.840 "
                    "FR1 curve; FR2 BWP scaling is not modelled - result is approximate.");
    }
    // Section 8.1.3 FR1 curve, expressed against the reference BWP (default 100 MHz):
    constexpr double kMinBwpMhz = 20.0;
    double span = static_cast<double>(m_refBwpMhz) - kMinBwpMhz; // 80 MHz for the 100 MHz ref
    double scale =
        (span > 0.0) ? 0.4 + 0.6 * (static_cast<double>(bandwidthMhz) - kMinBwpMhz) / span : 1.0;
    const double* tbl = (m_freqRange == FR2) ? UE_POWER_FR2 : UE_POWER_FR1;
    double floorFactor = UE_BWP_TRANSITION_FLOOR / tbl[NR_UE_PDCCH_PDSCH];
    return std::clamp(scale, floorFactor, 1.0);
}

void
NrUeEnergyModel::ApplyBwpScaling(uint32_t bandwidthMhz)
{
    NS_LOG_FUNCTION(this << bandwidthMhz);
    CommitOpenInterval(); // the scaling changes the current state's power
    m_bwpScale = ScaleBwp(bandwidthMhz);
    m_powerTrace = GetCurrentPowerW();
}

void
NrUeEnergyModel::ApplyAntennaScaling(uint32_t activeAntennas)
{
    NS_LOG_FUNCTION(this << activeAntennas);
    NS_ASSERT_MSG(activeAntennas >= 1, "At least one receive chain required");
    CommitOpenInterval(); // the scaling changes the current state's power
    // Each halving of the receive chains relative to the reference applies one
    // 0.7 factor (TR 38.840 Section 8.1.3: P_2Rx = 0.7 * P_4Rx, etc.).
    double factor = 1.0;
    uint32_t ant = GetRefRxAntennas();
    while (ant > activeAntennas && ant > 1)
    {
        factor *= 0.7;
        ant /= 2;
    }
    m_antennaScale = factor;
    m_powerTrace = GetCurrentPowerW();
}

void
NrUeEnergyModel::ApplyBdReduction(double alpha)
{
    NS_LOG_FUNCTION(this << alpha);
    NS_ASSERT_MSG(alpha > 0.0 && alpha <= 1.0, "alpha must be in (0, 1]");
    CommitOpenInterval(); // the reduction changes the current state's power
    // TR 38.840 Section 8.1.3: P(alpha) = alpha*Pt + (1-alpha)*0.7*Pt.
    m_bdScale = alpha + (1.0 - alpha) * 0.7;
    m_powerTrace = GetCurrentPowerW();
}

void
NrUeEnergyModel::SetUlTxPowerDbm(double txPowerDbm)
{
    NS_LOG_FUNCTION(this << txPowerDbm);
    CommitOpenInterval(); // the level changes the UL state's power
    if (m_freqRange == FR2)
    {
        m_ulRelativePower = UE_POWER_FR2[NR_UE_UL_TX];
        m_powerTrace = GetCurrentPowerW();
        return;
    }
    // FR1: linearly interpolate between the 0 dBm and 23 dBm anchors.
    double frac = std::clamp(txPowerDbm / 23.0, 0.0, 1.0);
    m_ulRelativePower =
        UE_UL_POWER_FR1_0DBM + frac * (UE_UL_POWER_FR1_23DBM - UE_UL_POWER_FR1_0DBM);
    m_powerTrace = GetCurrentPowerW();
}

void
NrUeEnergyModel::CommitOpenInterval()
{
    NS_LOG_FUNCTION(this);
    // Close the interval that has been running at the current power, before
    // anything changes that power. Callers must invoke this *before* mutating the
    // state or any scaling factor, so the elapsed time is charged at the power
    // that was actually in effect over it.
    Time now = Simulator::Now();
    double durationS = (now - m_lastUpdateTime).GetSeconds();
    m_totalEnergyJ = m_totalEnergyJ + GetCurrentPowerW() * durationS;
    m_stateTimeS[m_currentState] += durationS;
    m_lastUpdateTime = now;
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
}

bool
NrUeEnergyModel::IsSleepState(NrUePowerState state)
{
    return state == NR_UE_DEEP_SLEEP || state == NR_UE_LIGHT_SLEEP || state == NR_UE_MICRO_SLEEP;
}

double
NrUeEnergyModel::GetTransitionEnergyJ(NrUePowerState state) const
{
    // TR 38.840 Table 19 energy is [relative power x ms]. Convert to Joules:
    // relative x PowerUnit_mW -> mW, then mW x ms -> mW.ms -> J (factor 1e-6).
    const auto it = UE_TRANSITIONS.find(state);
    if (it == UE_TRANSITIONS.end())
    {
        return 0.0; // no tabulated transition: micro sleep, and every awake state
    }
    return it->second.energyRelMs * m_powerUnitMw * 1e-6;
}

void
NrUeEnergyModel::ChangeState(int newState)
{
    NS_LOG_FUNCTION(this << newState);
    NS_ASSERT_MSG(newState >= 0 && newState < NR_UE_NUM_STATES, "Invalid UE state");

    if (static_cast<NrUePowerState>(newState) == m_currentState)
    {
        return;
    }

    // Accrue the energy and the occupancy time of the state we are leaving.
    CommitOpenInterval();
    NrUePowerState oldState = m_currentState;
    m_currentState = static_cast<NrUePowerState>(newState);
    // TR 38.840 Table 19: entering a sleep state from a non-sleep state costs an
    // additional transition energy E over a total transition time T. Spread it as
    // an E/T transient rather than adding it as a lump to m_totalEnergyJ: a lump
    // is invisible to the attached EnergySource, which can only integrate the
    // current reported by DoGetCurrentA(), so the model and the source would
    // disagree by exactly E. Riding it on the transient keeps them consistent and
    // still totals P_sleep*T + E over the transition.
    if (IsSleepState(m_currentState) && !IsSleepState(oldState))
    {
        const auto it = UE_TRANSITIONS.find(m_currentState);
        Time transitionTime = (it != UE_TRANSITIONS.end()) ? it->second.totalTime : Seconds(0);
        if (transitionTime > Seconds(0))
        {
            TriggerTransition(GetTransitionEnergyJ(m_currentState) / transitionTime.GetSeconds(),
                              transitionTime);
        }
    }
    m_stateTrace = newState;
    m_powerTrace = GetCurrentPowerW();
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
    // The transient covers [trigger, end]. This value charges the interval that
    // ends at "now", so the bound is inclusive: an accrual landing exactly on the
    // end time still had the transient active for the whole interval it commits.
    // EndTransition() splits the interval there, so no interval ever straddles.
    if (Simulator::Now() <= m_transitionEndTime)
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
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
    m_transitionExtraW = extraPowerW;
    m_transitionEndTime = now + duration;
    m_powerTrace = GetCurrentPowerW();
    // Re-triggering while a transient is in flight extends it. Drop the pending
    // end event first: otherwise it would fire at the original end time and clear
    // the extended transient early.
    m_transitionEvent.Cancel();
    m_transitionEvent = Simulator::Schedule(duration, &NrUeEnergyModel::EndTransition, this);
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
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
    m_transitionExtraW = 0.0;
    m_transitionEndTime = now;
    m_powerTrace = GetCurrentPowerW();
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
    double durationS = (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    m_totalEnergyJ = m_totalEnergyJ + GetCurrentPowerW() * durationS;
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
    m_transitionEvent.Cancel();
    m_source = nullptr;
    energy::DeviceEnergyModel::DoDispose();
}

} // namespace ns3
