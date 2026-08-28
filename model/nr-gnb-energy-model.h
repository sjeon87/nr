// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5 - Energy consumption model for BS

#ifndef NR_GNB_ENERGY_MODEL_H
#define NR_GNB_ENERGY_MODEL_H

#include "ns3/device-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/traced-value.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup nr
 * @brief gNB power consumption states (TR 38.864 Table 5.1-2 / 5.1-3).
 */
enum class NrGnbPowerState
{
    DeepSleep,     //!< TR 38.864: Deep sleep  (P1)
    LightSleep,    //!< TR 38.864: Light sleep (P2)
    MicroSleep,    //!< TR 38.864: Micro sleep (P3) - static baseline
    ActiveDl,   //!< TR 38.864: Active DL   (P_DL formula)
    ActiveUl,   //!< TR 38.864: Active UL   (P_UL formula)
    Guard,      //!< DL/UL turnaround inside a slot. NOT a 3GPP sleep transition:
                //!< the symbol carries neither transmission nor reception, so it
                //!< is approximated at P3. Sleep transitions are charged as an
                //!< energy transient instead, see GetTransitionEnergyJ().
    NumStates   //!< sentinel
};

/**
 * @ingroup nr
 * @brief Per-symbol activity of the gNB radio within a slot (TR 38.864 Section 5.2).
 *
 * This classifies what the gNB is doing during an OFDM symbol, which is what the
 * energy formula keys off. It is NOT the 3GPP TDD slot type: 5G-LENA's "flexible"
 * (F) slot is a dually-schedulable slot (DL CTRL + DL data + UL data + UL CTRL),
 * not an idle one, and the scheduler resolves every allocated symbol to a
 * definite DL or UL direction before the slot executes. A symbol that no
 * var-TTI allocates carries no transmission or reception and is genuinely idle.
 */
enum class NrGnbSymbolType
{
    Dl,   //!< gNB transmitting DL - P_DL
    Ul,   //!< gNB receiving UL    - P_UL
    Idle, //!< Unallocated symbol  - P3
    NumTypes
};

/**
 * @ingroup nr
 * @brief TR 38.864 gNB power consumption model.
 *
 * Implements the BS energy model of TR 38.864 Section 5: the DL/UL active power
 * formulas with sa/sf/sp scaling, the P1..P5 relative power table, and
 * symbol-level slot energy accumulation (Section 5.2). It is an ns-3
 * energy::DeviceEnergyModel so it can be attached to an energy::EnergySource.
 *
 * Two complementary accounting paths feed the same running total:
 *   - Discrete state path: ChangeState() integrates the current state power
 *     over the elapsed time (used for sleep periods and whole-slot active
 *     intervals). This is also what drives the attached EnergySource.
 *   - Symbol-level path: UpdateSymbolPower() is called per OFDM symbol and
 *     FinalizeSlotEnergy() commits the per-symbol sum (Section 5.2). Use this
 *     for symbol-accurate active slots; do not mix it with ChangeState() over
 *     the same time interval or energy will be double counted.
 *
 * P1..P5 are stored as spec-derived tables keyed by (BsCategory, RefConfigSet).
 * These are two independent axes of TR 38.864 Table 5.1-3 and every combination
 * is tabulated; a single attribute default cannot encode them.
 */
class NrGnbEnergyModel : public energy::DeviceEnergyModel
{
  public:
    /**
     * @brief BS hardware category (TR 38.864 Section 5.1).
     */
    enum BsCategory
    {
        BsCat1, //!< BS Category 1 per TR 38.864 Tables 5.1-3 to 5.1-5
        BsCat2  //!< BS Category 2 per TR 38.864 Tables 5.1-3 to 5.1-5
    };

    /**
     * @brief Reference configuration set (TR 38.864 Table 5.1-1).
     */
    enum RefConfigSet
    {
        Custom, //!< No preset: every reference parameter as configured directly
        Set1,   //!< FR1 TDD, 100 MHz, 30 kHz SCS, 64 DL TxRU, 55 dBm
        Set2,   //!< FR1 FDD, 20 MHz, 15 kHz SCS, 32 DL TxRU, 49 dBm
        Set3    //!< FR2 TDD, 100 MHz, 120 kHz SCS, 2 DL TxRU, 33 dBm
    };

    /**
     * @brief PA efficiency mode (TR 38.864 Section 5.1).
     */
    enum EtaMode
    {
        EtaSingle, //!< eta = 1.0 for all sf, sp
        EtaDual    //!< eta = 1.0 if sf*sp >= 0.5, else 0.76
    };

    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrGnbEnergyModel constructor
     */
    NrGnbEnergyModel();

    /**
     * @brief ~NrGnbEnergyModel
     */
    ~NrGnbEnergyModel() override;

    // ----- 3GPP power formulas (pure, testable against TR 38.864 Section 5.1) -----

    /**
     * @brief Relative power for one of the P1..P5 states (current category/set).
     * @param state One of the discrete NrGnbPowerState values.
     * @return Relative power in power-units.
     */
    double GetRelativePower(NrGnbPowerState state) const;

    /**
     * @brief PA efficiency factor eta(sf, sp) (TR 38.864 Section 5.1).
     * @param sf Bandwidth utilization factor [0,1].
     * @param sp Tx power ratio [0,1].
     * @return 1.0 (EtaSingle, or sf*sp >= 0.5) or 0.76 (EtaDual and sf*sp < 0.5).
     */
    double GetEta(double sf, double sp) const;

    /**
     * @brief Instantaneous DL active power [W] (TR 38.864 Section 5.1).
     *
     * P_DL = P3 + sa*(P4 - P3) * [ A + (sf*sp / eta(sf,sp)) * (1 - A) ].
     *
     * @param sa Active TRxRU ratio [0,1].
     * @param sf Allocated-RB ratio [0,1].
     * @param sp Tx power ratio [0,1].
     * @return DL power in Watts.
     */
    double CalcDlPowerW(double sa, double sf, double sp) const;

    /**
     * @brief Instantaneous UL active power [W] (TR 38.864 Section 5.1).
     *
     * P_UL = P3 + sa*(P5 - P3). No sf/sp dependence.
     *
     * @param sa Active TRxRU ratio [0,1].
     * @return UL power in Watts.
     */
    double CalcUlPowerW(double sa) const;

    /**
     * @brief OFDM symbol duration for the configured numerology.
     * @return Symbol duration = (1ms / 2^mu) / 14.
     */
    Time GetSymbolDuration() const;

    /**
     * @brief Symbol-accurate energy of one slot (TR 38.864 Section 5.2).
     *
     * E_slot = sum_n P_symbol[n] * T_symbol, with P_symbol from the DL/UL/guard
     * classification. This is a pure helper (does not touch the running total);
     * use it to validate symbol-level accounting.
     *
     * @param pattern Per-symbol direction for the 14 symbols of the slot.
     * @param sa Active TRxRU ratio [0,1].
     * @param sf Allocated-RB ratio [0,1].
     * @param sp Tx power ratio [0,1].
     * @return Slot energy in Joules.
     */
    double ComputeSlotEnergyJ(const std::vector<NrGnbSymbolType>& pattern,
                              double sa,
                              double sf,
                              double sp) const;

    // ----- Symbol-level accumulation path (driven by NrGnbPhyEnergyListener) -----

    /**
     * @brief Accumulate one symbol's energy into the current slot (Section 5.2).
     * @param sa Active TRxRU ratio [0,1].
     * @param sf Allocated-RB ratio [0,1].
     * @param sp Tx power ratio [0,1].
     * @param symbolType DL / UL / guard classification of this symbol.
     */
    void UpdateSymbolPower(double sa, double sf, double sp, NrGnbSymbolType symbolType);

    /**
     * @brief Commit the accumulated per-symbol energy of the current slot.
     */
    void FinalizeSlotEnergy();

    // ----- Dynamic scaling-factor setters -----

    /**
     * @brief Set the bandwidth utilization factor sf for the active state.
     * @param sf Allocated-RB ratio [0,1].
     */
    void SetSf(double sf);

    /**
     * @brief Set the current Tx power; updates sp = P_tx_lin / P_ref_lin.
     * @param txPowerDbm Current total Tx power in dBm.
     */
    void SetTxPowerDbm(double txPowerDbm);

    // ----- State machine -----

    /**
     * @brief Change the gNB power state and accrue energy for the elapsed time.
     * @param newState New NrGnbPowerState (passed as int per DeviceEnergyModel).
     */
    void ChangeState(int newState) override;

    /**
     * @brief Get the current gNB power state.
     * @return The current NrGnbPowerState.
     */
    NrGnbPowerState GetCurrentState() const;

    /**
     * @brief Instantaneous power draw of the current discrete state [W].
     *
     * For ACTIVE_DL/UL this uses the current sa/sf/sp; for sleep states it is
     * the corresponding P1/P2/P3 relative value.
     *
     * @return Power in Watts.
     */
    double GetCurrentPowerW() const;

    /**
     * @brief Total energy consumed since simulation start [Joules].
     * @return Cumulative energy in Joules (includes the open state interval).
     */
    double GetTotalEnergyJ() const;

    /**
     * @brief True if a state is one of the three TR 38.864 sleep levels.
     * @param state The state to classify.
     * @return True for DeepSleep / LightSleep / MicroSleep.
     */
    static bool IsSleepState(NrGnbPowerState state);

    /**
     * @brief Additional transition energy for a sleep state [J].
     *
     * TR 38.864 Table 5.1-5, in (relative power) x (duration in ms), scaled by
     * PowerUnit. The tabulated value covers ramping down and ramping up together,
     * so it is charged once when the sleep state is entered rather than at both
     * ends. Micro sleep is immediate and costs nothing. This mirrors how
     * NrUeEnergyModel applies the TR 38.840 Table 19 transients.
     *
     * @param state The sleep state being entered (0 for non-sleep states).
     * @return Transition energy in Joules.
     */
    double GetTransitionEnergyJ(NrGnbPowerState state) const;

    /**
     * @brief Total transition time for a sleep state (TR 38.864 Table 5.1-4).
     *
     * Also covers ramping down and up together. A sleep interval shorter than
     * this does not pay for itself, so a DRX-style controller can use it to pick
     * a sleep depth.
     *
     * @param state The sleep state (zero for non-sleep states).
     * @return Total transition time.
     */
    Time GetTransitionTime(NrGnbPowerState state) const;

    /**
     * @brief Get the most recently computed Tx power ratio sp.
     *
     * sp = currentTxPower_linear / referenceTxPower_linear, computed by
     * SetTxPowerDbm() against the configured ReferenceTxPowerDbm attribute.
     *
     * @return Last sp value in [0.0, 1.0].
     */
    double GetSp() const;
    /**
     * @brief Set the OFDM symbol duration (sourced from the PHY by the listener).
     * @param symbolDuration Duration of one OFDM symbol.
     */
    void SetSymbolDuration(Time symbolDuration);

    // ----- energy::DeviceEnergyModel interface -----

    void SetEnergySource(Ptr<energy::EnergySource> source) override;
    double GetTotalEnergyConsumption() const override;
    void HandleEnergyDepletion() override;
    void HandleEnergyRecharged() override;
    void HandleEnergyChanged() override;

  protected:
    void DoDispose() override;
    void DoInitialize() override;

  private:
    /**
     * @brief Current draw of the current state, in Ampere (DeviceEnergyModel).
     * @return Current in Ampere (power / supply voltage).
     */
    double DoGetCurrentA() const override;

    /**
     * @brief Convert a relative power-unit value to Watts via PowerUnit.
     * @param relative Relative power in power-units.
     * @return Power in Watts.
     */
    double ToWatts(double relative) const;

    /**
     * @brief Reconcile the model with the selected TR 38.864 Table 5.1-1 set.
     *
     * No-op for Custom. Otherwise the set is authoritative for the reference Tx
     * power, which nothing else in the scenario supplies. The symbol duration is
     * validated against the set's SCS rather than overwritten: the PHY is the
     * source of truth for what is actually simulated, so a disagreement means the
     * scenario and the selected reference set describe different base stations,
     * and overwriting it would hide that.
     */
    void ApplyReferenceConfigSet();

    Ptr<energy::EnergySource> m_source; //!< Attached energy source (may be null)

    BsCategory m_bsCategory;  //!< BS hardware category
    RefConfigSet m_refConfig; //!< Reference configuration set
    EtaMode m_etaMode;        //!< PA efficiency mode
    double m_antennaRatioA;   //!< Antenna dynamic power fraction A
    double m_powerUnitW;      //!< Absolute scale: W per relative power-unit
    Time m_symbolDuration;    //!< OFDM symbol duration (set from NrGnbPhy::GetSymbolPeriod)
    double m_refTxPowerDbm;   //!< Reference Tx power for sp [dBm]
    bool m_symbolDurationFromPhy{false}; //!< SetSymbolDuration() was called by the listener
    double m_sa;              //!< Active TRxRU ratio; fixed at 1.0 until antenna muting is modelled
    double m_sf;              //!< Current bandwidth utilization factor
    double m_sp;              //!< Current Tx power ratio

    NrGnbPowerState m_currentState; //!< Current discrete power state
    Time m_lastUpdateTime;          //!< Time of last state change

    double m_slotEnergyAccumJ;   //!< Energy being accumulated for the slot about to start [J]
    double m_slotAccumDurationS; //!< Time span accumulated for that slot [s]
    double m_currentSlotPowerW;  //!< Average power of the slot currently elapsing [W]

    TracedValue<int> m_stateTrace;      //!< Fires on each discrete state change
    TracedValue<double> m_powerTrace;   //!< Fires with instantaneous power [W] on each change
    TracedValue<double> m_totalEnergyJ; //!< Accumulated energy [J], excl. open interval
};

} // namespace ns3

#endif // NR_GNB_ENERGY_MODEL_H
