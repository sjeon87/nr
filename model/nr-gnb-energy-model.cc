// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - BS power formulas and table 5.1-3
//   TR 38.864 V18.1.0 (2023-03): Section 5.2 - symbol-level time-domain energy

#include "nr-gnb-energy-model.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/energy-source.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbEnergyModel");
NS_OBJECT_ENSURE_REGISTERED(NrGnbEnergyModel);

// The TR 38.864 tables below are keyed lookups rather than positional arrays.
// Each is indexed by the thing the specification indexes it by - a BS category, a
// reference configuration set, a power state - so adding a row is one line and no
// enum ordering or array length has to agree with anything.
namespace
{
using BsCategory = NrGnbEnergyModel::BsCategory;
using RefConfigSet = NrGnbEnergyModel::RefConfigSet;

struct GnbPowerRow
{
    double p1; //!< Deep sleep
    double p2; //!< Light sleep
    double p3; //!< Micro sleep (also the static baseline)
    double p4; //!< Active DL
    double p5; //!< Active UL
};

// TR 38.864 Table 5.1-3, keyed by (BS category, reference configuration set).
// Deep sleep (P1) and light sleep (P2) are merged cells in the table, i.e. the
// same value across all three sets of a given BS category.
const std::map<std::pair<BsCategory, RefConfigSet>, GnbPowerRow> GNB_POWER_SETS{
    {{NrGnbEnergyModel::BsCat1, NrGnbEnergyModel::Set1}, {1.0, 25.0, 55.0, 280.0, 110.0}},
    {{NrGnbEnergyModel::BsCat1, NrGnbEnergyModel::Set2}, {1.0, 25.0, 50.0, 200.0, 90.0}},
    {{NrGnbEnergyModel::BsCat1, NrGnbEnergyModel::Set3}, {1.0, 25.0, 38.0, 152.0, 80.0}},
    {{NrGnbEnergyModel::BsCat2, NrGnbEnergyModel::Set1}, {1.0, 2.1, 5.5, 32.0, 6.5}},
    {{NrGnbEnergyModel::BsCat2, NrGnbEnergyModel::Set2}, {1.0, 2.1, 5.0, 26.0, 5.8}},
    {{NrGnbEnergyModel::BsCat2, NrGnbEnergyModel::Set3}, {1.0, 2.1, 3.0, 17.6, 4.2}},
};

struct GnbRefConfig
{
    double refTxPowerDbm; //!< Total DL power level
    uint32_t scsKhz;      //!< Subcarrier spacing
};

// TR 38.864 Table 5.1-1 reference configurations, keyed by the set itself so no
// index arithmetic ties the array order to the enum order. Only the members the
// model consumes are held: the reference Tx power, which nothing else supplies,
// and the SCS, which the PHY-derived symbol duration is checked against. For the
// record the full rows are:
//   Set 1: FR1 TDD,  100 MHz,  30 kHz, 1 TRP, 64 TxRU, 55 dBm
//   Set 2: FR1 FDD,   20 MHz,  15 kHz, 1 TRP, 32 TxRU, 49 dBm
//   Set 3: FR2 TDD,  100 MHz, 120 kHz, 1 TRP,  2 TxRU, 33 dBm
// Custom has no entry: it applies no preset, and callers guard for it.
const std::map<RefConfigSet, GnbRefConfig> GNB_REF_CONFIG{
    {NrGnbEnergyModel::Set1, {55.0, 30}},
    {NrGnbEnergyModel::Set2, {49.0, 15}},
    {NrGnbEnergyModel::Set3, {33.0, 120}},
};

struct GnbTransition
{
    double energyRelMs; //!< Table 5.1-5 additional transition energy
    Time totalTime;     //!< Table 5.1-4 total transition time
};

// TR 38.864 Table 5.1-4 (total transition time T) and Table 5.1-5 (additional
// transition energy E, in relative power x ms), keyed by (BS category, state).
// Both cover ramping down and ramping up together, so the energy is charged once
// on sleep entry. They are the same across reference sets for a given category.
// Only states that HAVE a transition appear: a lookup miss means zero, so no
// array length has to stay in step with the size of NrGnbPowerState.
const std::map<std::pair<BsCategory, NrGnbPowerState>, GnbTransition> GNB_TRANSITIONS{
    {{NrGnbEnergyModel::BsCat1, NrGnbPowerState::DeepSleep}, {1000.0, MilliSeconds(50)}},
    {{NrGnbEnergyModel::BsCat1, NrGnbPowerState::LightSleep}, {90.0, MilliSeconds(6)}},
    {{NrGnbEnergyModel::BsCat2, NrGnbPowerState::DeepSleep}, {17000.0, Seconds(10)}},
    {{NrGnbEnergyModel::BsCat2, NrGnbPowerState::LightSleep}, {1088.0, MilliSeconds(640)}},
    // Micro sleep is entered and left immediately: no transition time, no energy.
};
} // namespace

TypeId
NrGnbEnergyModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrGnbEnergyModel")
            .SetParent<energy::DeviceEnergyModel>()
            .SetGroupName("Nr")
            .AddConstructor<NrGnbEnergyModel>()
            .AddAttribute("BsCategory",
                          "BS hardware category (TR 38.864 Section 5.1).",
                          EnumValue(BsCat1),
                          MakeEnumAccessor<BsCategory>(&NrGnbEnergyModel::m_bsCategory),
                          MakeEnumChecker(BsCat1, "BsCat1", BsCat2, "BsCat2"))
            .AddAttribute(
                "RefConfigSet",
                "Named TR 38.864 Table 5.1-1 reference configuration. A named set "
                "is one coherent bundle: it fixes ReferenceTxPowerDbm and requires "
                "the PHY numerology to match the set's SCS. Custom applies no "
                "preset and leaves every reference parameter as configured.",
                EnumValue(Custom),
                MakeEnumAccessor<RefConfigSet>(&NrGnbEnergyModel::m_refConfig),
                MakeEnumChecker(Custom, "Custom", Set1, "Set1", Set2, "Set2", Set3, "Set3"))
            .AddAttribute("EtaMode",
                          "PA efficiency mode: single constant 1.0 or dual 0.76/1.0.",
                          EnumValue(EtaSingle),
                          MakeEnumAccessor<EtaMode>(&NrGnbEnergyModel::m_etaMode),
                          MakeEnumChecker(EtaSingle, "Single", EtaDual, "Dual"))
            .AddAttribute("AntennaRatio_A",
                          "Load-independent fraction A of the dynamic DL power in the "
                          "TR 38.864 v18.1.0 Section 5.1 base-station power equation "
                          "P_DL = P3 + sa*(P4-P3)*[A + (sf*sp/eta)*(1-A)]. Dimensionless "
                          "coefficient (NOT an antenna count).",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&NrGnbEnergyModel::m_antennaRatioA),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("PowerUnit",
                          "Absolute power of one TR 38.864 relative power-unit, in W. "
                          "PowerUnit is a user-supplied calibration assumption. 3GPP "
                          "provides only relative power, and does not provide the default "
                          "value for the absolute value of the unit. Hence the result "
                          "obtained with this default value do not represent the absolute "
                          "energy consumption per 3GPP model.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrGnbEnergyModel::m_powerUnitW),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ReferenceRbCount",
                          "Reference system bandwidth for sf, in RBs (TR 38.864 Section 5.1: "
                          "sf is the ratio between the RF bandwidth and the maximum system "
                          "BW). Set this to the RB count of the whole carrier. 0 means use "
                          "the RB count of the BWP reporting the allocation, which is only "
                          "correct when that BWP spans the entire carrier; with a narrower "
                          "BWP it makes sf ~= 1 at full load and hides the energy saving of "
                          "BWP adaptation (TR 38.864 Section 6.2.2).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrGnbEnergyModel::m_refRbCount),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NeglectUlDuringDl",
                          "TR 38.864 Section 5.1: \"For simultaneous DL and UL transmission "
                          "for FDD, the power for UL reception is neglected in this study.\" "
                          "True reproduces the study's assumption, which is what published "
                          "TR 38.864 results are based on. False adds the UL dynamic part on "
                          "top of the DL power instead - only the dynamic part, because P_UL "
                          "carries the same P3 baseline as P_DL. This is a study simplification "
                          "rather than physics: a real FDD base station does consume power "
                          "receiving while it transmits.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrGnbEnergyModel::m_neglectUlDuringDl),
                          MakeBooleanChecker())
            .AddAttribute("ReferenceTxPowerDbm",
                          "Reference total Tx power for sp [dBm] (TR 38.864 Table 5.1-1). "
                          "Ignored unless RefConfigSet is Custom, where a value that "
                          "contradicts the selected set is rejected.",
                          DoubleValue(55.0),
                          MakeDoubleAccessor(&NrGnbEnergyModel::m_refTxPowerDbm),
                          MakeDoubleChecker<double>())
            .AddTraceSource("State",
                            "Current discrete gNB power state (NrGnbPowerState as int).",
                            MakeTraceSourceAccessor(&NrGnbEnergyModel::m_stateTrace),
                            "ns3::TracedValueCallback::Int32")
            .AddTraceSource("InstantaneousPower",
                            "Instantaneous gNB power draw [W] at each state change.",
                            MakeTraceSourceAccessor(&NrGnbEnergyModel::m_powerTrace),
                            "ns3::TracedValueCallback::Double")
            .AddTraceSource("TotalEnergyConsumption",
                            "Total energy consumed by the gNB [J].",
                            MakeTraceSourceAccessor(&NrGnbEnergyModel::m_totalEnergyJ),
                            "ns3::TracedValueCallback::Double");
    return tid;
}

NrGnbEnergyModel::NrGnbEnergyModel()
    : m_source(nullptr),
      m_bsCategory(BsCat1),
      m_refConfig(Custom),
      m_etaMode(EtaSingle),
      m_antennaRatioA(0.4),
      m_powerUnitW(1.0),
      m_symbolDuration(Seconds((1e-3 / 2.0) / 14.0)), // mu=1 default until set
      m_refTxPowerDbm(55.0),
      m_refRbCount(0),
      m_neglectUlDuringDl(true),
      m_sa(1.0),
      m_sf(0.0),
      m_sp(1.0),
      m_currentState(NrGnbPowerState::MicroSleep),
      m_lastUpdateTime(Seconds(0)),
      m_slotEnergyAccumJ(0.0),
      m_slotAccumDurationS(0.0),
      m_currentPowerW(0.0),
      m_transitionExtraW(0.0),
      m_transitionEndTime(Seconds(0)),
      m_stateTrace(static_cast<int>(NrGnbPowerState::MicroSleep)),
      m_powerTrace(0.0),
      m_totalEnergyJ(0.0)
{
    NS_LOG_FUNCTION(this);
}

NrGnbEnergyModel::~NrGnbEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbEnergyModel::DoInitialize()
{
    ApplyReferenceConfigSet();
    // TR 38.864 Table 5.1-3 has no zero-power state: the BS draws at least P3 even
    // when idle. Seed the canonical power from the initial state so the source
    // drains from t=0 instead of only once the first slot has been finalized.
    m_currentPowerW = GetCurrentPowerW();
    m_powerTrace = m_currentPowerW;
    energy::DeviceEnergyModel::DoInitialize();
}

void
NrGnbEnergyModel::ApplyReferenceConfigSet()
{
    NS_LOG_FUNCTION(this);
    if (m_refConfig == Custom)
    {
        return;
    }
    const GnbRefConfig& cfg = GNB_REF_CONFIG.at(m_refConfig);
    // The set is authoritative for the reference Tx power: it is the rated power
    // of the BS class, which nothing in the scenario supplies.
    m_refTxPowerDbm = cfg.refTxPowerDbm;

    // The symbol duration, by contrast, comes from the PHY and describes what is
    // actually simulated. Check it against the set's SCS instead of overwriting
    // it, so a scenario that does not match the chosen reference is reported
    // rather than silently reinterpreted.
    if (!m_symbolDurationFromPhy)
    {
        return;
    }
    Time expected = Seconds((1e-3 / (cfg.scsKhz / 15.0)) / 14.0);
    // Time is quantized to integer nanoseconds; numerologies differ by factors of
    // two, so a 1% band separates them while tolerating that rounding.
    double got = m_symbolDuration.GetSeconds();
    NS_ABORT_MSG_IF(std::abs(got - expected.GetSeconds()) > 0.01 * expected.GetSeconds(),
                    "PHY symbol duration " << got << " s does not match the selected TR 38.864 "
                                           << "reference set (" << cfg.scsKhz << " kHz SCS, "
                                           << expected.GetSeconds() << " s). The scenario and the "
                                           << "reference set describe different base stations; use "
                                           << "RefConfigSet=Custom to configure them separately.");
}

double
NrGnbEnergyModel::GetRelativePower(NrGnbPowerState state) const
{
    NS_LOG_FUNCTION(this << static_cast<int>(state));
    // A P1..P5 row is always required, so Custom (no named set) uses the Set 1
    // row, which is what the model used before named sets became opt-in.
    const RefConfigSet set = (m_refConfig == Custom) ? Set1 : m_refConfig;
    const GnbPowerRow& row = GNB_POWER_SETS.at({m_bsCategory, set});

    switch (state)
    {
    case NrGnbPowerState::DeepSleep:
        return row.p1;
    case NrGnbPowerState::LightSleep:
        return row.p2;
    case NrGnbPowerState::MicroSleep:
        return row.p3;
    case NrGnbPowerState::ActiveDl:
        return row.p4;
    case NrGnbPowerState::ActiveUl:
        return row.p5;
    case NrGnbPowerState::Guard:
        // A DL/UL turnaround symbol carries no transmission or reception, so it is
        // charged at the micro-sleep level. This is NOT the TR 38.864 Table 5.1-4 /
        // 5.1-5 sleep transition, which is an energy transient, not a power level.
        return row.p3;
    default:
        NS_ABORT_MSG("Invalid gNB power state");
    }
}

bool
NrGnbEnergyModel::IsSleepState(NrGnbPowerState state)
{
    return state == NrGnbPowerState::DeepSleep || state == NrGnbPowerState::LightSleep ||
           state == NrGnbPowerState::MicroSleep;
}

bool
NrGnbEnergyModel::IsDeepOrLightSleep(NrGnbPowerState state)
{
    return state == NrGnbPowerState::DeepSleep || state == NrGnbPowerState::LightSleep;
}

double
NrGnbEnergyModel::GetTransitionEnergyJ(NrGnbPowerState state) const
{
    NS_LOG_FUNCTION(this << static_cast<int>(state));
    const auto it = GNB_TRANSITIONS.find({m_bsCategory, state});
    if (it == GNB_TRANSITIONS.end())
    {
        return 0.0; // no tabulated transition: active states, and micro sleep
    }
    // Table 5.1-5 energy is [relative power x ms]. Convert to Joules: relative x
    // PowerUnit_W -> W, then W x ms -> W.ms -> J (factor 1e-3).
    return it->second.energyRelMs * m_powerUnitW * 1e-3;
}

Time
NrGnbEnergyModel::GetTransitionTime(NrGnbPowerState state) const
{
    NS_LOG_FUNCTION(this << static_cast<int>(state));
    const auto it = GNB_TRANSITIONS.find({m_bsCategory, state});
    return (it != GNB_TRANSITIONS.end()) ? it->second.totalTime : Seconds(0);
}

double
NrGnbEnergyModel::ToWatts(double relative) const
{
    NS_LOG_FUNCTION(this << relative);
    return relative * m_powerUnitW;
}

double
NrGnbEnergyModel::GetEta(double sf, double sp) const
{
    NS_LOG_FUNCTION(this << sf << sp);
    if (m_etaMode == EtaSingle)
    {
        return 1.0;
    }
    return (sf * sp >= 0.5) ? 1.0 : 0.76;
}

double
NrGnbEnergyModel::CalcDlPowerW(double sa, double sf, double sp) const
{
    NS_LOG_FUNCTION(this << sa << sf << sp);
    // TR 38.864 Section 5.1:
    //   P_DL = P3 + sa*(P4 - P3) * [ A + (sf*sp / eta) * (1 - A) ]
    double p3 = GetRelativePower(NrGnbPowerState::MicroSleep);
    double p4 = GetRelativePower(NrGnbPowerState::ActiveDl);
    double eta = GetEta(sf, sp);
    double rel =
        p3 + sa * (p4 - p3) * (m_antennaRatioA + (sf * sp / eta) * (1.0 - m_antennaRatioA));
    return ToWatts(rel);
}

double
NrGnbEnergyModel::CalcUlPowerW(double sa) const
{
    NS_LOG_FUNCTION(this << sa);
    // TR 38.864 Section 5.1: P_UL = P3 + sa*(P5 - P3).
    double p3 = GetRelativePower(NrGnbPowerState::MicroSleep);
    double p5 = GetRelativePower(NrGnbPowerState::ActiveUl);
    return ToWatts(p3 + sa * (p5 - p3));
}

void
NrGnbEnergyModel::SetSymbolDuration(Time symbolDuration)
{
    NS_LOG_FUNCTION(this << symbolDuration);
    m_symbolDuration = symbolDuration;
    m_symbolDurationFromPhy = true;
}

Time
NrGnbEnergyModel::GetSymbolDuration() const
{
    return m_symbolDuration;
}

double
NrGnbEnergyModel::ComputeSlotEnergyJ(const std::vector<NrGnbSymbolType>& pattern,
                                     double sa,
                                     double sf,
                                     double sp) const
{
    NS_LOG_FUNCTION(this << sa << sf << sp);
    double tSym = GetSymbolDuration().GetSeconds();
    double energyJ = 0.0;
    for (NrGnbSymbolType sym : pattern)
    {
        double powerW = 0.0;
        switch (sym)
        {
        case NrGnbSymbolType::Dl:
            powerW = CalcDlPowerW(sa, sf, sp);
            break;
        case NrGnbSymbolType::Ul:
            powerW = CalcUlPowerW(sa);
            break;
        case NrGnbSymbolType::Idle:
            powerW = ToWatts(GetRelativePower(NrGnbPowerState::MicroSleep));
            break;
        default:
            NS_ABORT_MSG("Invalid symbol type");
        }
        energyJ += powerW * tSym;
    }
    return energyJ;
}

void
NrGnbEnergyModel::UpdateSymbolPower(double sa, double sf, double sp, NrGnbSymbolType symbolType)
{
    NS_LOG_FUNCTION(this << sa << sf << sp << static_cast<int>(symbolType));
    double tSym = GetSymbolDuration().GetSeconds();
    double powerW = 0.0;
    switch (symbolType)
    {
    case NrGnbSymbolType::Dl:
        powerW = CalcDlPowerW(sa, sf, sp);
        break;
    case NrGnbSymbolType::Ul:
        powerW = CalcUlPowerW(sa);
        break;
    case NrGnbSymbolType::Idle:
        powerW = ToWatts(GetRelativePower(NrGnbPowerState::MicroSleep));
        break;
    default:
        NS_ABORT_MSG("Invalid symbol type");
    }
    m_slotEnergyAccumJ += powerW * tSym;
    m_slotAccumDurationS += tSym;
}

void
NrGnbEnergyModel::FinalizeSlotEnergy()
{
    NS_LOG_FUNCTION(this << m_slotEnergyAccumJ);
    // Close the elapsed interval at the power of the slot that was in progress,
    // and refresh the source while that power is still installed. Installing the
    // new slot power first would make the source charge the finished interval at
    // the incoming slot's power, i.e. an off-by-one-slot error.
    CommitInterval();
    // In deep or light sleep the BS is not transmitting or receiving, so the
    // symbol path has nothing to say about its power: keep the Table 5.1-3 sleep
    // power that ChangeState() installed. Both paths write m_currentPowerW and
    // this is what stops them contradicting each other.
    // Micro sleep is deliberately excluded: per TR 38.864 Table 5.1-3 it is the
    // static baseline the BS sits at while awake, and it is the model's initial
    // state, so treating it as "asleep" would pin the gNB at P3 for the whole
    // simulation and discard the symbol-level accounting entirely.
    if (!IsDeepOrLightSleep(m_currentState))
    {
        // TR 38.864 Section 5.1: "the power consumption in a slot is the sum of the
        // power consumption associated with symbols in the slot". The slot average
        // is energy-equivalent over the slot, so exposing it as a constant power
        // makes the source integrate exactly what the per-symbol sum accounts for.
        m_currentPowerW =
            (m_slotAccumDurationS > 0.0) ? (m_slotEnergyAccumJ / m_slotAccumDurationS) : 0.0;
    }
    m_slotEnergyAccumJ = 0.0;
    m_slotAccumDurationS = 0.0;
    m_powerTrace = GetInstantaneousPowerW();
}

double
NrGnbEnergyModel::GetInstantaneousPowerW() const
{
    // The transient covers [entry, end]; see CommitInterval() on why the bound is
    // inclusive. EndTransition() splits the interval there, so none straddles.
    return m_currentPowerW + ((Simulator::Now() <= m_transitionEndTime) ? m_transitionExtraW : 0.0);
}

void
NrGnbEnergyModel::CommitInterval()
{
    NS_LOG_FUNCTION(this);
    Time now = Simulator::Now();
    m_totalEnergyJ =
        m_totalEnergyJ + GetInstantaneousPowerW() * (now - m_lastUpdateTime).GetSeconds();
    m_lastUpdateTime = now;
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
}

void
NrGnbEnergyModel::EndTransition()
{
    NS_LOG_FUNCTION(this);
    CommitInterval(); // charged at state power + transient, which is still installed
    m_transitionExtraW = 0.0;
    m_transitionEndTime = Simulator::Now();
    m_powerTrace = GetInstantaneousPowerW();
}

void
NrGnbEnergyModel::ReportBwpOccupancy(uint32_t bwpIndex, const BwpOccupancy& occupancy)
{
    NS_LOG_FUNCTION(this << bwpIndex);
    // Close the elapsed interval at the power that was actually drawn over it,
    // before the new record can change it. When several BWPs of the same carrier
    // report at the same instant, every commit after the first spans zero time,
    // so the intermediate partial aggregates cost nothing and the final power is
    // the same whatever order the listeners fire in.
    // Every bandwidth part of a carrier must share its numerology. RB width is
    // SCS x 12, so RB counts at different subcarrier spacings are not the same
    // unit and the carrier total below would be meaningless; the symbol grids
    // would not line up either, and the model carries a single symbol duration.
    // This is a configuration error and it shows up on the very first slot.
    for (const auto& [idx, existing] : m_bwpOccupancy)
    {
        if (idx == bwpIndex || existing.symbolDuration == Seconds(0))
        {
            continue;
        }
        NS_ABORT_MSG_IF(occupancy.symbolDuration != Seconds(0) &&
                            occupancy.symbolDuration != existing.symbolDuration,
                        "Bandwidth parts of one carrier must share a numerology: BWP "
                            << bwpIndex << " reports a symbol duration of "
                            << occupancy.symbolDuration << " while BWP " << idx << " reports "
                            << existing.symbolDuration
                            << ". Give them the same numerology, or model them as "
                               "separate carriers.");
    }

    CommitInterval();
    m_bwpOccupancy[bwpIndex] = occupancy;
    m_currentPowerW = EvaluateCarrierPowerW();
    m_powerTrace = GetInstantaneousPowerW();
}

double
NrGnbEnergyModel::EvaluateCarrierPowerW() const
{
    NS_LOG_FUNCTION(this);
    const Time now = Simulator::Now();

    // Gather the live records. Positions matter: the carrier is transmitting in a
    // symbol if ANY of its bandwidth parts is, so the active span is the union of
    // their masks. Counts alone cannot express that - two BWPs each active for
    // seven symbols may overlap completely or not at all, and the two cases carry
    // very different energy.
    struct Live
    {
        uint16_t dlDataMask;
        uint16_t dlCtrlMask;
        uint16_t ulMask;
        double dlDataRegPerSym;
        double dlCtrlRegPerSym;
        double txPowerLin;
    };

    std::vector<Live> live;
    uint32_t symbolsPerSlot = 0;
    uint32_t carrierRbFromBwps = 0;

    for (const auto& [idx, o] : m_bwpOccupancy)
    {
        if (o.validUntil > Seconds(0) && now > o.validUntil)
        {
            continue; // stale: this BWP stopped reporting
        }
        symbolsPerSlot = std::max(symbolsPerSlot, o.symbolsPerSlot);

        // Only downlink-capable bandwidth parts belong to the DL reference
        // bandwidth. Including an uplink-only BWP would cap sf below 1 and make
        // P4 unreachable, contradicting Table 5.1-2, which defines Active DL as
        // the P4 state.
        if (o.dlCapable)
        {
            carrierRbFromBwps += o.rbCount;
        }

        const auto dlDataSym = static_cast<double>(std::popcount(o.dlDataMask));
        const auto dlCtrlSym = static_cast<double>(std::popcount(o.dlCtrlMask));
        live.push_back({o.dlDataMask,
                        o.dlCtrlMask,
                        o.ulMask,
                        dlDataSym > 0 ? o.dlDataReg / dlDataSym : 0.0,
                        dlCtrlSym > 0 ? o.dlCtrlReg / dlCtrlSym : 0.0,
                        o.txPowerLin});
    }

    if (symbolsPerSlot == 0 || live.empty())
    {
        return 0.0; // nothing reported yet
    }

    const uint32_t carrierRb = (m_refRbCount > 0) ? m_refRbCount : carrierRbFromBwps;
    // Same linear scale the bandwidth parts report their own power on, so the ratio
    // below is dimensionless and the absolute unit cancels.
    const double refTxPowerLin = std::pow(10.0, m_refTxPowerDbm / 10.0);
    const double tSym = GetSymbolDuration().GetSeconds();
    const double p3W = ToWatts(GetRelativePower(NrGnbPowerState::MicroSleep));
    double energyJ = 0.0;

    // TR 38.864 Section 5.2: "the power consumption in a slot is the sum of the
    // power consumption associated with symbols in the slot". Walk the symbols and
    // classify each from the union of the masks.
    for (uint32_t sym = 0; sym < symbolsPerSlot && sym < 16; ++sym)
    {
        const auto bit = static_cast<uint16_t>(1U << sym);
        double occupiedReg = 0.0;
        double occupiedPowerLin = 0.0;
        bool anyDl = false;
        bool anyUl = false;

        for (const auto& l : live)
        {
            bool bwpTransmitting = false;
            if (l.dlDataMask & bit)
            {
                occupiedReg += l.dlDataRegPerSym;
                anyDl = true;
                bwpTransmitting = true;
            }
            if (l.dlCtrlMask & bit)
            {
                occupiedReg += l.dlCtrlRegPerSym;
                anyDl = true;
                bwpTransmitting = true;
            }
            // Only a bandwidth part actually transmitting in this symbol puts power
            // on the air, so only those contribute to the carrier's Tx power.
            if (bwpTransmitting)
            {
                occupiedPowerLin += l.txPowerLin;
            }
            anyUl = anyUl || ((l.ulMask & bit) != 0);
        }

        if (anyDl)
        {
            // The carrier is transmitting. sf is the fraction of the carrier
            // occupied in THIS symbol, so no averaging assumption is involved.
            const double sf = (carrierRb > 0) ? std::min(1.0, occupiedReg / carrierRb) : 0.0;

            // sp is a carrier property too. Tx power is configured per bandwidth
            // part, but what the specification scales by is the power of the DL
            // transmission as a whole against the reference configuration, so
            // summing the parts transmitting in this symbol is the sp analogue of
            // summing their RBs for sf. Falls back to the model-wide value when no
            // part reported a power, which is the single-BWP and unit-test path.
            double sp = m_sp;
            if (occupiedPowerLin > 0.0 && refTxPowerLin > 0.0)
            {
                sp = occupiedPowerLin / refTxPowerLin;
                if (sp > 1.0)
                {
                    NS_LOG_WARN("Carrier Tx power exceeds the reference configuration by "
                                << sp << "x; clamping sp to 1. Check ReferenceTxPowerDbm "
                                << "against the per-BWP TxPower values.");
                    sp = 1.0;
                }
            }
            double p = CalcDlPowerW(m_sa, sf, sp);

            // TR 38.864 Section 5.1: "For simultaneous DL and UL transmission for
            // FDD, the power for UL reception is neglected in this study." That is
            // an explicit study assumption rather than physics, so it can be turned
            // off - in which case only the UL *dynamic* part is added, because
            // P_UL already carries the same P3 baseline as P_DL and adding both
            // would count it twice.
            if (anyUl && !m_neglectUlDuringDl)
            {
                p += ToWatts(m_sa * (GetRelativePower(NrGnbPowerState::ActiveUl) -
                                     GetRelativePower(NrGnbPowerState::MicroSleep)));
            }
            energyJ += p * tSym;
        }
        else if (anyUl)
        {
            energyJ += CalcUlPowerW(m_sa) * tSym;
        }
        else
        {
            energyJ += p3W * tSym;
        }
    }

    const double slotDurationS = symbolsPerSlot * tSym;
    return (slotDurationS > 0.0) ? (energyJ / slotDurationS) : 0.0;
}

void
NrGnbEnergyModel::SetSf(double sf)
{
    NS_LOG_FUNCTION(this << sf);
    m_sf = std::clamp(sf, 0.0, 1.0);
}

void
NrGnbEnergyModel::SetTxPowerDbm(double txPowerDbm)
{
    NS_LOG_FUNCTION(this << txPowerDbm);
    double curLin = std::pow(10.0, txPowerDbm / 10.0);
    double refLin = std::pow(10.0, m_refTxPowerDbm / 10.0);
    m_sp = (refLin > 0.0) ? std::clamp(curLin / refLin, 0.0, 1.0) : 1.0;
}

double
NrGnbEnergyModel::GetSp() const
{
    return m_sp;
}

double
NrGnbEnergyModel::CalcSf(uint32_t usedReg, uint32_t symbols, uint32_t bwpRbCount) const
{
    // TR 38.864 Section 5.1: sf is "the ratio between the RF bandwidth and the maximum
    // system BW". The denominator is therefore the carrier's reference bandwidth, not
    // the bandwidth of the reporting BWP: normalising against the reporting BWP makes
    // the numerator and denominator shrink together, so a fully used BWP always yields
    // sf ~= 1 no matter how narrow it is. That is exactly what would make BWP
    // adaptation (Section 6.2.2) report no saving at all.
    const uint32_t refRb = (m_refRbCount > 0) ? m_refRbCount : bwpRbCount;
    if (refRb == 0 || symbols == 0)
    {
        return 0.0;
    }
    const double raw = static_cast<double>(usedReg) / (static_cast<double>(refRb) * symbols);
    // Clamped, because the ratio can legitimately exceed 1: control REGs are summed
    // over DCIs, and several UEs' PDCCH can occupy the same RBs in the same symbol.
    // A ratio far above 1 more likely means ReferenceRbCount is too small to span the
    // carrier, which the clamp would otherwise hide entirely.
    if (raw > 1.0)
    {
        NS_LOG_WARN("sf raw ratio " << raw << " > 1 (" << usedReg << " REGs over " << symbols
                                    << " symbols against " << refRb
                                    << " reference RBs); clamping to 1");
    }
    return std::min(1.0, raw);
}

void
NrGnbEnergyModel::ChangeState(int newState)
{
    NS_LOG_FUNCTION(this << newState);
    NS_ASSERT_MSG(newState >= 0 && newState < static_cast<int>(NrGnbPowerState::NumStates),
                  "Invalid gNB state");

    if (static_cast<NrGnbPowerState>(newState) == m_currentState)
    {
        return;
    }

    // Close the elapsed interval at the outgoing power, refreshing the source
    // while it is still installed, before anything below changes it.
    CommitInterval();
    NrGnbPowerState oldState = m_currentState;
    m_currentState = static_cast<NrGnbPowerState>(newState);
    m_currentPowerW = GetCurrentPowerW();

    // TR 38.864 Table 5.1-4 / 5.1-5: entering a sleep state costs an additional
    // transition energy E spread over a total transition time T, during which
    // "relative power of sleep mode i is assumed to be consumed" (Section 5.1).
    // So the interval draws the sleep power plus E/T, totalling P_sleep*T + E.
    // Spreading it rather than adding a lump is both what the spec describes and
    // what an EnergySource can actually integrate: a lump added straight to the
    // running total would never reach the source, which only sees current.
    if (IsDeepOrLightSleep(m_currentState) && !IsDeepOrLightSleep(oldState))
    {
        Time transitionTime = GetTransitionTime(m_currentState);
        if (transitionTime > Seconds(0))
        {
            m_transitionExtraW = GetTransitionEnergyJ(m_currentState) / transitionTime.GetSeconds();
            m_transitionEndTime = Simulator::Now() + transitionTime;
            m_transitionEvent.Cancel();
            m_transitionEvent =
                Simulator::Schedule(transitionTime, &NrGnbEnergyModel::EndTransition, this);
        }
    }

    m_stateTrace = newState;
    m_powerTrace = GetInstantaneousPowerW();
}

NrGnbPowerState
NrGnbEnergyModel::GetCurrentState() const
{
    return m_currentState;
}

double
NrGnbEnergyModel::GetCurrentPowerW() const
{
    NS_LOG_FUNCTION(this);
    switch (m_currentState)
    {
    case NrGnbPowerState::ActiveDl:
        return CalcDlPowerW(m_sa, m_sf, m_sp);
    case NrGnbPowerState::ActiveUl:
        return CalcUlPowerW(m_sa);
    default:
        return ToWatts(GetRelativePower(m_currentState));
    }
}

double
NrGnbEnergyModel::GetTotalEnergyJ() const
{
    NS_LOG_FUNCTION(this);
    // Energy committed so far, plus the not-yet-committed part of the interval in
    // progress. CommitInterval() is the only writer of m_totalEnergyJ and it always
    // advances m_lastUpdateTime, so the two parts abut and never overlap.
    // m_slotEnergyAccumJ is deliberately NOT added: it is the next slot's energy,
    // installed as power at its own FinalizeSlotEnergy().
    double durationS = (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    return m_totalEnergyJ + GetInstantaneousPowerW() * durationS;
}

void
NrGnbEnergyModel::SetEnergySource(Ptr<energy::EnergySource> source)
{
    NS_LOG_FUNCTION(this << source);
    NS_ASSERT(source);
    m_source = source;
}

double
NrGnbEnergyModel::GetTotalEnergyConsumption() const
{
    return GetTotalEnergyJ();
}

void
NrGnbEnergyModel::HandleEnergyDepletion()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbEnergyModel::HandleEnergyRecharged()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbEnergyModel::HandleEnergyChanged()
{
    NS_LOG_FUNCTION(this);
}

double
NrGnbEnergyModel::DoGetCurrentA() const
{
    NS_LOG_FUNCTION(this);
    double voltage = m_source ? m_source->GetSupplyVoltage() : 1.0;
    return (voltage > 0.0) ? GetInstantaneousPowerW() / voltage : 0.0;
}

void
NrGnbEnergyModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_transitionEvent.Cancel();
    m_source = nullptr;
    energy::DeviceEnergyModel::DoDispose();
}

} // namespace ns3
