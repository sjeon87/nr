// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#ifndef NR_UE_ENERGY_MODEL_H
#define NR_UE_ENERGY_MODEL_H

#include "ns3/device-energy-model.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/traced-value.h"

namespace ns3
{

/**
 * @ingroup nr
 * @brief UE energy consumption states (TR 38.840 Table 18, Section 8.1).
 *
 * Relative power units where Deep Sleep = 1. The absolute power of each state
 * is the relative value times the PowerUnit attribute (mW per unit).
 */
enum NrUePowerState
{
    NR_UE_DEEP_SLEEP,  //!< TR 38.840: "Deep Sleep"  - timing not maintained
    NR_UE_LIGHT_SLEEP, //!< TR 38.840: "Light Sleep" - timing maintained
    NR_UE_MICRO_SLEEP, //!< TR 38.840: "Micro sleep" - immediate transition
    NR_UE_PDCCH_ONLY,  //!< TR 38.840: "PDCCH-only"  - control monitoring
    NR_UE_SSB_CSI_RS,  //!< TR 38.840: "SSB or CSI-RS proc."
    NR_UE_PDCCH_PDSCH, //!< TR 38.840: "PDCCH + PDSCH" - full DL reception
    NR_UE_UL_TX,       //!< TR 38.840: "UL" - uplink transmission
    NR_UE_NUM_STATES   //!< sentinel, not a real state
};

/**
 * @ingroup nr
 * @brief TR 38.840 UE power consumption model.
 *
 * Implements the relative power-state model of TR 38.840 Section 8: the model
 * holds a current power state and, on every state change, integrates the
 * energy spent in the previous state (E = P * dt) into a running total. As an
 * ns-3 energy::DeviceEnergyModel it can be attached to a standard
 * energy::EnergySource and depleted over the course of a simulation.
 *
 * The per-state relative power values are spec-derived tables keyed by
 * frequency range (FR1/FR2); FreqRange and PowerUnit are the user-facing
 * knobs. The BWP / antenna / PDCCH blind-decoding scalings of Section 8.1.3
 * are cached multipliers applied to the active DL reception states only;
 * sleep and UL states are left unscaled. Dynamic scaling is BWP-based:
 * NrUePhyEnergyListener re-applies ApplyBwpScaling() whenever the active BWP
 * bandwidth changes.
 */
class NrUeEnergyModel : public energy::DeviceEnergyModel
{
  public:
    /**
     * @brief Frequency range selector (TR 38.840 Section 8.1.1 / 8.1.2).
     */
    enum FreqRange
    {
        FR1, //!< Sub-6 GHz reference configuration
        FR2  //!< mmWave reference configuration
    };

    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrUeEnergyModel constructor
     */
    NrUeEnergyModel();

    /**
     * @brief ~NrUeEnergyModel
     */
    ~NrUeEnergyModel() override;

    // ----- 3GPP power lookup (pure, testable against TR 38.840 Table 18) -----

    /**
     * @brief Relative power for a state in the configured frequency range.
     *
     * Returns the unscaled TR 38.840 Table 18 relative value (Deep Sleep = 1).
     *
     * @param state The power state to look up.
     * @return Relative power in power-units.
     */
    double GetRelativePower(NrUePowerState state) const;

    /**
     * @brief Absolute power [W] for a state, including any active-state scaling.
     *
     * P_W = GetRelativePower(state) * PowerUnit_mW * 1e-3, with the BWP /
     * antenna / blind-decoding multiplier applied to the active DL reception
     * states (PDCCH_ONLY, SSB_CSI_RS, PDCCH_PDSCH).
     *
     * @param state The power state to evaluate.
     * @return Absolute power in Watts.
     */
    double GetStatePowerW(NrUePowerState state) const;

    // ----- Scaling factors (TR 38.840 Section 8.1.3), pure helpers -----

    /**
     * @brief BWP bandwidth scaling factor (TR 38.840 Section 8.1.3).
     *
     * scale(X) = 0.4 + 0.6 * (X - 20) / 80, valid for X in {10,20,40,80,100} MHz
     * with linear interpolation between. Result is not allowed to scale the
     * active power below the BWP transition floor (50 power-units).
     *
     * This is the spec curve on its own: const and free of side effects, so the
     * formula can be asserted directly against the TR 38.840 anchors without
     * mutating the model. ApplyBwpScaling() is the command that caches it.
     *
     * @param bandwidthMhz Active BWP bandwidth in MHz.
     * @return Multiplicative scaling factor in (0, 1].
     */
    double ScaleBwp(uint32_t bandwidthMhz) const;

    /**
     * @brief Cache the ScaleBwp() factor for the active reception states.
     *
     * The command half of the pair above: called by NrUePhyEnergyListener
     * whenever the active BWP bandwidth changes, so a BWP switch is reflected in
     * the UE power draw.
     *
     * @param bandwidthMhz New active BWP bandwidth in MHz.
     */
    void ApplyBwpScaling(uint32_t bandwidthMhz);

    /**
     * @brief Cache the antenna scaling factor (TR 38.840 Section 8.1.3).
     *
     * FR1: P_2Rx = 0.7 * P_4Rx. FR2: P_1Rx = 0.7 * P_2Rx. Each halving of the
     * active receive chains relative to the reference applies one 0.7 factor.
     *
     * Takes powered RF receive chains, NOT the MIMO rank: spatial layers are
     * unrelated to how many chains are on, and Table 21 scales with the latter.
     *
     * @param activeAntennas Number of powered receive chains (>= 1).
     */
    void ApplyAntennaScaling(uint32_t activeAntennas);

    /**
     * @brief Cache the PDCCH blind-decoding reduction factor (Section 8.1.3).
     *
     * P(alpha) = alpha * Pt + (1 - alpha) * 0.7 * Pt, i.e. multiplier
     * = alpha + (1 - alpha) * 0.7, for alpha in (0, 1].
     *
     * @param alpha Ratio of reduced PDCCH candidates to the reference maximum.
     */
    void ApplyBdReduction(double alpha);

    // ----- State machine -----

    /**
     * @brief Change the UE power state and accrue energy for the elapsed time.
     * @param newState New NrUePowerState (passed as int per DeviceEnergyModel).
     */
    void ChangeState(int newState) override;

    /**
     * @brief Get the current UE power state.
     * @return The current NrUePowerState.
     */
    NrUePowerState GetCurrentState() const;

    /**
     * @brief Instantaneous power draw of the current state.
     * @return Power in Watts.
     */
    double GetCurrentPowerW() const;

    /**
     * @brief Total energy consumed since simulation start [Joules].
     *
     * Includes the energy accrued in the current state interval.
     *
     * @return Cumulative energy in Joules.
     */
    double GetTotalEnergyJ() const;

    /**
     * @brief Fraction of time spent in a given state since tracking start.
     *
     * Used for TR 38.840 occupancy evaluation. Includes the current open
     * interval. Tracking starts at the first ChangeState() call (or the
     * explicit ResetOccupancy()).
     *
     * @param state The state to query.
     * @return Time fraction in [0, 1].
     */
    double GetStateTimeFraction(NrUePowerState state) const;

    /**
     * @brief Average relative power over the tracked window [power-units].
     *
     * sum_state ( occupancy(state) * relativePower(state) ), with the active DL
     * states' relative power scaled by the same BWP/antenna/blind-decoding
     * factors GetStatePowerW() applies, so this matches the power actually
     * accounted rather than the unscaled TR 38.840 table value. This is the
     * quantity TR 38.840 reports for power-saving comparisons.
     *
     * @return Average relative power in power-units.
     */
    double GetAverageRelativePower() const;

    /**
     * @brief Reset the occupancy accounting window to start now.
     */
    void ResetOccupancy();

    /**
     * @brief Select the UL transmit power level (TR 38.840 Table 18).
     *
     * Linearly interpolates the UL relative power between the 0 dBm and 23 dBm
     * anchors (FR1). FR2 uses a single UL value so the level has no effect.
     *
     * @param txPowerDbm UL transmit power in dBm.
     */
    void SetUlTxPowerDbm(double txPowerDbm);

    // ----- Transition transients -----

    /**
     * @brief Fire the configured setup/RRC transient (SetupTransitionPower/Time).
     *
     * Called by NrUePhyEnergyListener when the UE wakes from deep sleep on a
     * transport block. This is the public entry point: the transient is
     * configured through the SetupTransitionPower / SetupTransitionTime
     * attributes, so callers do not supply the values themselves. No-op if
     * SetupTransitionPower is 0.
     */
    void TriggerSetupTransition();

    // ----- energy::DeviceEnergyModel interface -----

    void SetEnergySource(Ptr<energy::EnergySource> source) override;
    double GetTotalEnergyConsumption() const override;
    void HandleEnergyDepletion() override;
    void HandleEnergyRecharged() override;
    void HandleEnergyChanged() override;

  protected:
    void DoDispose() override;

  private:
    /**
     * @brief Current draw of the current state, in Ampere (DeviceEnergyModel).
     * @return Current in Ampere (power / supply voltage).
     */
    double DoGetCurrentA() const override;

    /**
     * @brief Reference receive-chain count, resolving the 0 = derive sentinel.
     * @return m_refRxAntennas, or the FreqRange default if it is still unset.
     */
    uint32_t GetRefRxAntennas() const;

    /**
     * @brief True if a state is an active DL reception state (gets scaled).
     * @param state The state to classify.
     * @return True for PDCCH_ONLY / SSB_CSI_RS / PDCCH_PDSCH.
     */
    static bool IsActiveDlState(NrUePowerState state);

    /**
     * @brief Charge a transition transient (e.g. the RRC connection-setup spike).
     *
     * Adds @p extraPowerW on top of the current state power for @p duration,
     * then settles back, modelling the high-power burst hardware shows on a
     * sleep->active transition. This is the general mechanism; the transients
     * the model actually fires are driven through TriggerSetupTransition(), so
     * that the values stay owned by the attributes rather than by callers.
     *
     * @param extraPowerW Additional transient power [W].
     * @param duration    How long the transient lasts.
     */
    void TriggerTransition(double extraPowerW, Time duration);

    /**
     * @brief Commit the interval running at the current power, and advance.
     *
     * Must be called *before* changing the state or any scaling factor, so the
     * elapsed time is charged at the power actually in effect over it. Without
     * it, a mid-state change to a scaling factor would retroactively re-price the
     * whole open interval, and GetTotalEnergyJ() could even decrease.
     */
    void CommitOpenInterval();

    /**
     * @brief End the current transition transient: commit its energy, settle.
     */
    void EndTransition();

    /**
     * @brief True if a state is one of the three TR 38.840 sleep states.
     * @param state The state to classify.
     * @return True for DEEP_SLEEP / LIGHT_SLEEP / MICRO_SLEEP.
     */
    static bool IsSleepState(NrUePowerState state);

    /**
     * @brief TR 38.840 Table 19 additional transition energy for a sleep state.
     * @param state The sleep state being entered.
     * @return Additional transition energy [J]; 0 for non-sleep / micro sleep.
     */
    double GetTransitionEnergyJ(NrUePowerState state) const;

    Ptr<energy::EnergySource> m_source; //!< Attached energy source (may be null)

    FreqRange m_freqRange;     //!< FR1 or FR2 power table selector
    double m_powerUnitMw;      //!< Absolute scale: mW per relative power-unit
    uint32_t m_refRxAntennas;  //!< Reference receive antennas (TR 38.840 8.1.3)
    uint32_t m_activeRxChains; //!< Powered receive chains; 0 = same as reference
    uint32_t m_refBwpMhz;      //!< Reference BWP bandwidth in MHz (100)

    NrUePowerState m_currentState; //!< Current power state
    Time m_lastUpdateTime;         //!< Time of last state change
    double m_ulRelativePower;      //!< Current UL relative power (level-dependent)

    double m_stateTimeS[NR_UE_NUM_STATES]; //!< Cumulative time in each state [s]
    Time m_occupancyStart;                 //!< Start of the occupancy window

    double m_bwpScale;     //!< Cached BWP scaling factor for active states
    double m_antennaScale; //!< Cached antenna scaling factor for active states
    double m_bdScale;      //!< Cached blind-decoding reduction factor

    double m_transitionExtraW;      //!< Transient extra power during a transition [W]
    Time m_transitionEndTime;       //!< End time of the current transition transient
    EventId m_transitionEvent;      //!< Pending EndTransition event (cancelled on re-trigger)
    double m_setupTransitionPowerW; //!< Default setup/RRC transient power [W] (0 = off)
    Time m_setupTransitionTime;     //!< Default setup/RRC transient duration

    TracedValue<int> m_stateTrace;      //!< Fires on each state change
    TracedValue<double> m_powerTrace;   //!< Fires with instantaneous power [W] on each change
    TracedValue<double> m_totalEnergyJ; //!< Accumulated energy [J], excl. open interval

    void DoInitialize() override;
};

} // namespace ns3

#endif // NR_UE_ENERGY_MODEL_H
