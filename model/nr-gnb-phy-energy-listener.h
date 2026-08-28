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
     * BWP RBs and the slot symbol count used to classify the reported symbols.
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

    /**
     * @brief Whether this bandwidth part can carry downlink.
     *
     * Derived from the PHY's TDD pattern when SetPhy() attaches. The model keeps
     * a bandwidth part that cannot transmit out of the carrier's downlink
     * reference bandwidth: counting it would cap sf below 1 and put P4 out of
     * reach, contradicting TR 38.864 Table 5.1-2.
     *
     * @return True unless the pattern is uplink only.
     */
    bool IsDlCapable() const;

    /**
     * @brief Per-slot gNB driver, connected to NrGnbPhy "SlotEnergyStats".
     *
     * Translates one slot's reported occupancy into an NrGnbEnergyModel
     * BwpOccupancy record (TR 38.864 Section 5.2). The symbol masks say WHICH
     * symbols carry DL data, DL control and UL, so the model can classify each
     * symbol from the union across the carrier's bandwidth parts:
     *   - DL data / DL control (PDCCH) -> DL active power at the measured sf;
     *   - UL data / UL control (PUCCH/SRS) -> UL power, which has no sf/sp term;
     *   - unallocated -> micro sleep (P3).
     * "Idle" is not the 3GPP "flexible" (F) slot type: F is a dually-schedulable
     * slot already resolved to DL/UL directions, so a fully-scheduled F slot has
     * no idle symbols at all.
     *
     * Public so a test can drive one slot directly, as NrUePhyEnergyListener
     * does with its own trace sinks. Production code never calls this: SetPhy()
     * connects it to the PHY's trace source.
     *
     * @param sfnSf       The current system frame / slot number.
     * @param availableRb Resource blocks available in the BWP.
     * @param dlDataMask  Symbols carrying DL data, one bit per symbol.
     * @param dlDataReg   DL data REGs (RB x symbols) over the slot.
     * @param ulMask      Symbols carrying UL data or UL control.
     * @param dlCtrlMask  Symbols carrying DL control (PDCCH).
     * @param dlCtrlReg   DL control REGs (RB x symbols) over the slot.
     * @param bwpId       BWP id, which keys the record inside the model.
     * @param cellId      Cell id.
     */
    void SlotEnergyStatsCallback(const SfnSf& sfnSf,
                                 uint32_t availableRb,
                                 uint16_t dlDataMask,
                                 uint32_t dlDataReg,
                                 uint16_t ulMask,
                                 uint16_t dlCtrlMask,
                                 uint32_t dlCtrlReg,
                                 uint16_t bwpId,
                                 uint16_t cellId);

  protected:
    /**
     * @brief Release attached pointers. Inherited from Object.
     */
    void DoDispose() override;

  private:
    /**
     * @brief Push the gNB's current Tx power to the energy model and cache sp.
     *
     * The energy model owns the configured 3GPP reference Tx power, so it
     * computes sp; this only reports the current Tx power and reads sp back.
     * Tx power can change at runtime, so this is called per slot.
     */
    void RefreshSp();

    Ptr<NrGnbPhy> m_phy;           //!< Attached gNB PHY (may be null)
    Ptr<NrGnbEnergyModel> m_model; //!< Attached gNB energy model (may be null)

    double m_lastDlSf; //!< Last DL sf computed: used_REGs / (available_RBs * used_sym)
    double m_lastSp;   //!< Last sp computed: currentTxPower_lin / refTxPower_lin
    double m_lastSa;   //!< Last sa: activeTRxRUs / totalTRxRUs. Fixed at 1.0
                       //!< until antenna muting is added (no source in PHY yet).

    uint32_t m_totalBwpRbs;    //!< Total RBs in active BWP (from NrGnbPhy)
    bool m_dlCapable;          //!< This BWP can transmit DL (from the PHY slot pattern)
    uint32_t m_symbolsPerSlot; //!< OFDM symbols per slot (from NrGnbPhy, 12 or 14)
};

} // namespace ns3

#endif // NR_GNB_PHY_ENERGY_LISTENER_H
