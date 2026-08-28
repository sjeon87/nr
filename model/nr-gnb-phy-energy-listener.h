// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - Energy consumption model for BS

#ifndef NR_GNB_PHY_ENERGY_LISTENER_H
#define NR_GNB_PHY_ENERGY_LISTENER_H

#include "sfnsf.h"

#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

namespace ns3
{

// Forward declarations
class NrGnbPhy;
class NrGnbEnergyModel;

/**
 * @ingroup nr
 * @brief Callback bridge between NR gNB PHY layer events and the gNB energy model.
 *
 * This class is the decoupling layer between the gNB's PHY event stream and the
 * gNB energy model state machine. It subscribes to the existing NrGnbPhy
 * "SlotEnergyStats" trace, which reports the per-slot scheduler usage split by
 * direction (DL/UL data and DL/UL control symbols and REGs), extracts the
 * dynamic scaling factors (sa, sf, sp) required by the 3GPP TR 38.864 power
 * formula, and drives NrGnbEnergyModel symbol-by-symbol, charging DL symbols at
 * the DL active power and UL symbols at the UL power (TR 38.864 Section 5.2).
 *
 * Design rationale:
 *   - This class must NOT be aware of the energy formula internals.
 *     It only knows: "event occurred, extract parameters, notify model."
 *   - It must be usable both with and without an energy model installed
 *     (if no model is attached, callbacks are no-ops).
 *   - It has a single responsibility: the gNB side of the mapping. The UE side
 *     lives in a separate listener.
 */
class NrGnbPhyEnergyListener : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrGnbPhyEnergyListener constructor
     */
    NrGnbPhyEnergyListener();
    /**
     * @brief ~NrGnbPhyEnergyListener
     */
    ~NrGnbPhyEnergyListener() override;

    /**
     * @brief Attach this listener to a gNB PHY instance.
     *
     * Subscribes to the NrGnbPhy "SlotEnergyStats" trace and caches the total
     * BWP RBs and the reference Tx power for the sp computation.
     *
     * @param phy Pointer to the NrGnbPhy on the gNB node.
     */
    void SetPhy(Ptr<NrGnbPhy> phy);

    /**
     * @brief Connect the gNB energy model that this listener will drive.
     * @param model Pointer to the NrGnbEnergyModel instance.
     */
    void SetEnergyModel(Ptr<NrGnbEnergyModel> model);

    /**
     * @brief Get the most recently computed DL bandwidth utilization sf.
     *
     * sf = used_REGs / (available_RBs * used_symbols)
     * Source: TR 38.864 Section 5.1 definition of sf.
     *
     * @return Last sf value in [0.0, 1.0].
     */
    double GetLastDlSf() const;

    /**
     * @brief Get the most recently computed UL bandwidth utilization sf.
     * @return Last UL sf value in [0.0, 1.0].
     */
    double GetLastUlSf() const;

    /**
     * @brief Get the most recently computed Tx power ratio sp.
     *
     * sp = currentTxPower_linear / referenceTxPower_linear
     * Source: TR 38.864 Section 5.1 definition of sp.
     *
     * @return Last sp value in [0.0, 1.0].
     */
    double GetLastSp() const;

    /**
     * @brief Get the most recently computed active antenna ratio sa.
     *
     * sa = activeTRxRUs / totalTRxRUs
     * Source: TR 38.864 Section 5.1 definition of sa.
     *
     * @return Last sa value in [0.0, 1.0].
     */
    double GetLastSa() const;

  protected:
    /**
     * @brief Release attached pointers. Inherited from Object.
     */
    void DoDispose() override;

  private:
    /**
     * @brief Per-slot gNB driver, connected to NrGnbPhy "SlotEnergyStats".
     *
     * Builds a direction-aware per-symbol power timeline (TR 38.864 Section 5.2):
     *   - DL data / DL control (PDCCH) symbols -> DL active power at the measured
     *     sf (the gNB is transmitting, PA on);
     *   - UL data / UL control (PUCCH/SRS) symbols -> UL power (the gNB is
     *     receiving), which has no sf/sp dependence;
     *   - unallocated symbols -> micro-sleep (P3).
     * "Idle" is not the 3GPP "flexible" (F) slot type: F is a dually-schedulable
     * slot already resolved to DL/UL directions, so a fully-scheduled F slot has
     * no idle symbols at all. After the 14 symbols the slot energy is committed.
     *
     * @param sfnSf       The current system frame / slot number.
     * @param availableRb Resource blocks available in the BWP.
     * @param dlDataSym   DL data symbols in the slot.
     * @param dlDataReg   DL data REGs (RB x symbols), for the DL data sf.
     * @param ulDataSym   UL data symbols in the slot.
     * @param dlCtrlSym   DL control (PDCCH) symbols.
     * @param dlCtrlReg   DL control REGs (RB x symbols), for the PDCCH sf.
     * @param ulCtrlSym   UL control (PUCCH/SRS) symbols.
     * @param bwpId       BWP id.
     * @param cellId      Cell id.
     */
    void SlotEnergyStatsCallback(const SfnSf& sfnSf,
                                 uint32_t availableRb,
                                 uint32_t dlDataSym,
                                 uint32_t dlDataReg,
                                 uint32_t ulDataSym,
                                 uint32_t dlCtrlSym,
                                 uint32_t dlCtrlReg,
                                 uint32_t ulCtrlSym,
                                 uint16_t bwpId,
                                 uint16_t cellId);

    /**
     * @brief Recompute m_lastSp from the gNB's current Tx power.
     *
     * sp = currentTxPower_lin / referenceTxPower_lin. Tx power can change at
     * runtime, so this is called per slot rather than computed once.
     */
    void RefreshSp();

    Ptr<NrGnbPhy> m_phy;           //!< Attached gNB PHY (may be null)
    Ptr<NrGnbEnergyModel> m_model; //!< Attached gNB energy model (may be null)

    double m_lastDlSf; //!< Last DL sf computed: used_REGs / (available_RBs * used_sym)
    double m_lastUlSf; //!< Last UL sf computed
    double m_lastSp;   //!< Last sp computed: currentTxPower_lin / refTxPower_lin
    double m_lastSa;   //!< Last sa: activeTRxRUs / totalTRxRUs. Fixed at 1.0
                       //!< until antenna muting is added (no source in PHY yet).

    uint32_t m_totalBwpRbs;       //!< Total RBs in active BWP (from NrGnbPhy)
    double m_referenceTxPowerDbm; //!< Reference Tx power for sp computation
};

} // namespace ns3

#endif // NR_GNB_PHY_ENERGY_LISTENER_H
