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

#include "ns3/double.h"
#include "ns3/energy-source.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbEnergyModel");
NS_OBJECT_ENSURE_REGISTERED(NrGnbEnergyModel);

// TR 38.864 Table 5.1-3 relative power values, indexed [P1..P5]. BS category and
// reference configuration set are two independent axes of the table and every
// combination is tabulated, which a single ns-3 attribute default cannot express,
// so they live in a lookup table.
namespace
{
struct GnbPowerRow
{
    double p1; //!< Deep sleep
    double p2; //!< Light sleep
    double p3; //!< Micro sleep (also the static baseline)
    double p4; //!< Active DL
    double p5; //!< Active UL
};

// TR 38.864 Table 5.1-3. Deep sleep (P1) and light sleep (P2) are merged cells in
// the table, i.e. the same value across all three sets of a given BS category.
constexpr GnbPowerRow CAT1_SET1 = {1.0, 25.0, 55.0, 280.0, 110.0};
constexpr GnbPowerRow CAT1_SET2 = {1.0, 25.0, 50.0, 200.0, 90.0};
constexpr GnbPowerRow CAT1_SET3 = {1.0, 25.0, 38.0, 152.0, 80.0};
constexpr GnbPowerRow CAT2_SET1 = {1.0, 2.1, 5.5, 32.0, 6.5};
constexpr GnbPowerRow CAT2_SET2 = {1.0, 2.1, 5.0, 26.0, 5.8};
constexpr GnbPowerRow CAT2_SET3 = {1.0, 2.1, 3.0, 17.6, 4.2};

// TR 38.864 Table 5.1-1 reference configurations, indexed to RefConfigSet - 1
// (Custom has no entry). Only the members the model actually consumes are held:
// the reference Tx power, which nothing else supplies, and the SCS, which the
// PHY-derived symbol duration is checked against. For the record the full rows
// are:
//   Set 1: FR1 TDD,  100 MHz,  30 kHz, 1 TRP, 64 TxRU, 55 dBm
//   Set 2: FR1 FDD,   20 MHz,  15 kHz, 1 TRP, 32 TxRU, 49 dBm
//   Set 3: FR2 TDD,  100 MHz, 120 kHz, 1 TRP,  2 TxRU, 33 dBm
struct GnbRefConfig
{
    double refTxPowerDbm; //!< Total DL power level
    uint32_t scsKhz;      //!< Subcarrier spacing
};

constexpr GnbRefConfig GNB_REF_CONFIG[] = {
    {55.0, 30},  // Set1
    {49.0, 15},  // Set2
    {33.0, 120}, // Set3
};

// TR 38.864 Table 5.1-4 (total transition time T) and Table 5.1-5 (additional
// transition energy E, in relative power x ms), indexed by NrGnbPowerState.
// Both cover ramping down and ramping up together, so the energy is charged once
// on sleep entry. They are the same across reference sets for a given category.
// (Time is not constexpr-constructible, so this is a runtime-const array.)
struct GnbTransition
{
    double energyRelMs; //!< Table 5.1-5 additional transition energy
    Time totalTime;     //!< Table 5.1-4 total transition time
};

const GnbTransition CAT1_TRANSITION[] = {
    {1000.0, MilliSeconds(50)}, // DeepSleep
    {90.0, MilliSeconds(6)},    // LightSleep
    {0.0, MilliSeconds(0)},     // MicroSleep (immediate)
};

const GnbTransition CAT2_TRANSITION[] = {
    {17000.0, Seconds(10)},      // DeepSleep
    {1088.0, MilliSeconds(640)}, // LightSleep
    {0.0, MilliSeconds(0)},      // MicroSleep (immediate)
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
            .AddAttribute("RefConfigSet",
                          "Named TR 38.864 Table 5.1-1 reference configuration. A named set "
                          "is one coherent bundle: it fixes ReferenceTxPowerDbm and requires "
                          "the PHY numerology to match the set's SCS. Custom applies no "
                          "preset and leaves every reference parameter as configured.",
                          EnumValue(Custom),
                          MakeEnumAccessor<RefConfigSet>(&NrGnbEnergyModel::m_refConfig),
                          MakeEnumChecker(Custom,
                                          "Custom",
                                          Set1,
                                          "Set1",
                                          Set2,
                                          "Set2",
                                          Set3,
                                          "Set3"))
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
      m_sa(1.0),
      m_sf(0.0),
      m_sp(1.0),
      m_currentState(NrGnbPowerState::MicroSleep),
      m_lastUpdateTime(Seconds(0)),
      m_slotEnergyAccumJ(0.0),
      m_currentSlotPowerW(0.0),
      m_slotAccumDurationS(0.0),
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
    const GnbRefConfig& cfg = GNB_REF_CONFIG[m_refConfig - 1];
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
    GnbPowerRow row;
    if (m_bsCategory == BsCat1)
    {
        row = (m_refConfig == Set2) ? CAT1_SET2 : (m_refConfig == Set3) ? CAT1_SET3 : CAT1_SET1;
    }
    else
    {
        row = (m_refConfig == Set2) ? CAT2_SET2 : (m_refConfig == Set3) ? CAT2_SET3 : CAT2_SET1;
    }

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

double
NrGnbEnergyModel::GetTransitionEnergyJ(NrGnbPowerState state) const
{
    NS_LOG_FUNCTION(this << static_cast<int>(state));
    if (!IsSleepState(state))
    {
        return 0.0;
    }
    const GnbTransition* table = (m_bsCategory == BsCat1) ? CAT1_TRANSITION : CAT2_TRANSITION;
    // Table 5.1-5 energy is [relative power x ms]. Convert to Joules: relative x
    // PowerUnit_W -> W, then W x ms -> W.ms -> J (factor 1e-3).
    return table[static_cast<int>(state)].energyRelMs * m_powerUnitW * 1e-3;
}

Time
NrGnbEnergyModel::GetTransitionTime(NrGnbPowerState state) const
{
    NS_LOG_FUNCTION(this << static_cast<int>(state));
    if (!IsSleepState(state))
    {
        return Seconds(0);
    }
    const GnbTransition* table = (m_bsCategory == BsCat1) ? CAT1_TRANSITION : CAT2_TRANSITION;
    return table[static_cast<int>(state)].totalTime;
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
    Time now = Simulator::Now();
    // Commit only the interval that has actually elapsed since the last update,
    // charged at the power of the slot that was in progress. never in advance.
    m_totalEnergyJ = m_totalEnergyJ + m_currentSlotPowerW * (now - m_lastUpdateTime).GetSeconds();
    m_lastUpdateTime = now;
    m_currentSlotPowerW =
        (m_slotAccumDurationS > 0.0) ? (m_slotEnergyAccumJ / m_slotAccumDurationS) : 0.0;
    m_slotEnergyAccumJ = 0.0;
    m_slotAccumDurationS = 0.0;

    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
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

void
NrGnbEnergyModel::ChangeState(int newState)
{
    NS_LOG_FUNCTION(this << newState);
    NS_ASSERT_MSG(newState >= 0 && newState < static_cast<int>(NrGnbPowerState::NumStates),
                  "Invalid gNB state");

    Time now = Simulator::Now();
    double durationS = (now - m_lastUpdateTime).GetSeconds();
    m_totalEnergyJ = m_totalEnergyJ + GetCurrentPowerW() * durationS;
    m_lastUpdateTime = now;
    NrGnbPowerState oldState = m_currentState;
    m_currentState = static_cast<NrGnbPowerState>(newState);
    // TR 38.864 Table 5.1-5: charge the additional transition energy once when a
    // sleep state is entered from a non-sleep state. The tabulated value already
    // covers ramping down and up, so it is not charged again on wake-up.
    if (IsSleepState(m_currentState) && !IsSleepState(oldState))
    {
        m_totalEnergyJ = m_totalEnergyJ + GetTransitionEnergyJ(m_currentState);
    }

    m_stateTrace = newState;
    m_powerTrace = GetCurrentPowerW();
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
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
    // Energy for already-elapsed slots, plus the fraction of the in-progress slot
    // that has elapsed so far. m_slotEnergyAccumJ is deliberately NOT added: it is
    // the next slot's energy, committed at its own FinalizeSlotEnergy().
    double durationS = (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    return m_totalEnergyJ + m_currentSlotPowerW * durationS;
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
    return (voltage > 0.0) ? GetCurrentPowerW() / voltage : 0.0;
}

void
NrGnbEnergyModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_source = nullptr;
    energy::DeviceEnergyModel::DoDispose();
}

} // namespace ns3
