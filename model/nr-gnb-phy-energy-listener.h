// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - Energy consumption model for BS

#ifndef NR_GNB_PHY_ENERGY_LISTENER_H
#define NR_GNB_PHY_ENERGY_LISTENER_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/sfnsf.h"
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
 * gNB energy model state machine. It subscribes to trace sources on NrGnbPhy,
 * extracts the dynamic scaling factors (sa, sf, sp) required by the 3GPP
 * TR 38.864 power formula, and forwards energy model state-change calls to
 * NrGnbEnergyModel.
 *
 * Design rationale:
 *   - This class must NOT be aware of the energy formula internals.
 *     It only knows: "event occurred, extract parameters, notify model."
 *   - It must be usable both with and without an energy model installed
 *     (if no model is attached, callbacks are no-ops).
 *   - It has a single responsibility: the gNB side of the mapping. The UE side
 *     lives in a separate listener.
 *
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
     * Subscribes to the following trace sources on phy:
     *   - SlotIndication     - fires each slot start. extract TDD structure here
     *   - PacketBurstSent    - DL burst sent; extract active RBs for sf
     *   - UlRxSuccess        - UL reception; triggers UL power state
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
     * sf = allocated_RBs_DL / total_BWP_RBs
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
     * @brief Called each time a new slot starts (gNB side).
     *
     * Responsibilities:
     *   1. Extract TDD symbol pattern for this slot from NrGnbPhy.
     *   2. Compute sf = allocated_DL_RBs / total_BWP_RBs (from scheduler output).
     *   3. Compute sp from current Tx power setting.
     *   4. Call NrGnbEnergyModel::UpdateSymbolPower() for each of the 14 symbols.
     *
     * @param sfnSf The current system frame / slot number.
     */
    void SlotIndicationCallback(const SfnSf& sfnSf);

    /**
     * @brief Called when a DL packet burst is sent by the gNB.
     *
     * Used to extract the number of allocated DL RBs for sf computation.
     * sf = allocatedRbs / m_totalBwpRbs
     *
     * @param allocatedRbs Number of DL resource blocks allocated in this TTI.
     */
    void DlBurstSentCallback(uint32_t allocatedRbs);

    /**
     * @brief Called when a UL burst reception completes at the gNB.
     *
     * Used to extract the number of allocated UL RBs for the UL sf factor.
     *
     * @param allocatedRbs Number of UL resource blocks in this TTI.
     */
    void UlReceiveCallback(uint32_t allocatedRbs);

    /**
     * @brief Recompute m_lastSp from the gNB's current Tx power.
     *
     * sp = currentTxPower_lin / referenceTxPower_lin. Tx power can change at
     * runtime, so this is called per slot rather than computed once.
     */
    void RefreshSp();

    Ptr<NrGnbPhy> m_phy;             //!< Attached gNB PHY (may be null)
    Ptr<NrGnbEnergyModel> m_model;   //!< Attached gNB energy model (may be null)

    double m_lastDlSf; //!< Last DL sf computed: allocated_RBs / total_RBs
    double m_lastUlSf; //!< Last UL sf computed
    double m_lastSp;   //!< Last sp computed: currentTxPower_lin / refTxPower_lin
    double m_lastSa;   //!< Last sa: activeTRxRUs / totalTRxRUs. Fixed at 1.0
                       //!< until antenna muting is added (no source in PHY yet).

    uint32_t m_totalBwpRbs;         //!< Total RBs in active BWP (from NrGnbPhy)
    double m_referenceTxPowerDbm;   //!< Reference Tx power for sp computation
};

} // namespace ns3

#endif // NR_GNB_PHY_ENERGY_LISTENER_H
