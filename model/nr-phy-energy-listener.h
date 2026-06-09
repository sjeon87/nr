// Copyright (c) 2024 University of Moratuwa
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - Energy consumption model for BS
//   TR 38.840 V16.0.0 (2019-06): Section 8   - UE energy consumption evaluation

#ifndef NR_PHY_ENERGY_LISTENER_H
#define NR_PHY_ENERGY_LISTENER_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/sfnsf.h"
namespace ns3
{

// Forward declarations
class NrUePhy;
class NrGnbPhy;
class NrUeEnergyModel;
class NrGnbEnergyModel;

/**
 * @ingroup nr
 * @brief Callback bridge between NR PHY layer events and the NR energy models.
 *
 * This class is the decoupling layer between the simulation's PHY event
 * stream and the energy model state machines. It subscribes to trace
 * sources on NrUePhy and NrGnbPhy, extracts the dynamic scaling factors
 * (sa, sf, sp) required by the 3GPP TR 38.864 power formula, and forwards
 * energy model state-change calls to NrUeEnergyModel and NrGnbEnergyModel.
 *
 * Design rationale:
 *   - This class must NOT be aware of the energy formula internals.
 *     It only knows: "event occurred, extract parameters, notify model."
 *   - It must be usable both with and without an energy model installed
 *     (if no model is attached, callbacks are no-ops).
 *
 */
class NrPhyEnergyListener : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrPhyEnergyListener constructor
     */
    NrPhyEnergyListener();
    /**
     * @brief ~NrPhyEnergyListener
     */
    ~NrPhyEnergyListener() override;

    /**
     * @brief Attach this listener to a UE PHY instance.
     *
     * Subscribes to the following trace sources on phy:
     *   - SlotIndication     - fires each slot start
     *   - PhyRxCtrlEndOk     - PDCCH successfully decoded (state -> PDCCH_ONLY or PDCCH_PDSCH)
     *   - PhyTxEnd           - UL transmission completed (state -> back to PDCCH or sleep)
     *   - DlHarqFeedback     - HARQ ACK/NACK for state tracking
     *
     * @param phy Pointer to the NrUePhy on the UE node.
     */
    void SetUePhy(Ptr<NrUePhy> phy);

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
    void SetGnbPhy(Ptr<NrGnbPhy> phy);

    /**
     * @brief Connect the UE energy model that this listener will drive.
     * @param model Pointer to the NrUeEnergyModel instance.
     */
    void SetUeEnergyModel(Ptr<NrUeEnergyModel> model);

    /**
     * @brief Connect the gNB energy model that this listener will drive.
     * @param model Pointer to the NrGnbEnergyModel instance.
     */
    void SetGnbEnergyModel(Ptr<NrGnbEnergyModel> model);

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

  private:
    /**
     * @brief Called each time a new slot starts (UE side).
     *
     * Responsibilities:
     *   1. Determine UE DRX state from MAC DRX timer state.
     *   2. Set UE energy model state to PDCCH_ONLY if in DRX ON.
     *   3. Set UE energy model state to MICRO_SLEEP/LIGHT_SLEEP if in DRX OFF.
     *
     * @param sfnSf The current system frame / slot number.
     */
    void UeSlotIndicationCallback(const SfnSf& sfnSf);

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
    void GnbSlotIndicationCallback(const SfnSf& sfnSf);

    /**
     * @brief Called when PDCCH decoding succeeds on the UE.
     *
     * Transitions UE energy model:
     *   - If DCI has DL grant: state -> NR_UE_PDCCH_PDSCH 
     *   - If DCI has UL grant: state will transition to NR_UE_UL_TX at grant time
     *   - Otherwise:           state -> NR_UE_PDCCH_ONLY 
     *
     * @param hasDownlinkGrant True if the decoded DCI contains a PDSCH grant.
     * @param hasUplinkGrant True if the decoded DCI contains a PUSCH grant.
     */
    void UePdcchDecodeSuccessCallback(bool hasDownlinkGrant, bool hasUplinkGrant);

    /**
     * @brief Called when a DL packet burst is sent by the gNB.
     *
     * Used to extract the number of allocated DL RBs for sf computation.
     * sf = allocatedRbs / m_totalBwpRbs
     *
     * @param allocatedRbs Number of DL resource blocks allocated in this TTI.
     */
    void GnbDlBurstSentCallback(uint32_t allocatedRbs);

    /**
     * @brief Called when a UL burst reception completes at the gNB.
     *
     * Used to extract the number of allocated UL RBs for the UL sf factor.
     *
     * @param allocatedRbs Number of UL resource blocks in this TTI.
     */
    void GnbUlReceiveCallback(uint32_t allocatedRbs);

    /**
     * @brief Called when UL transmission starts on the UE.
     *
     * Transitions UE energy model state to NR_UE_UL_TX.
     * Power depends on Tx power level:
     *   - If txPowerDbm <= 0 dBm  -> UlPower0dBm
     *   - If txPowerDbm > 0 dBm   -> interpolate or use UlPower23dBm
     *   Source: TR 38.840 Table 18.
     *
     * @param txPowerDbm Current UL transmission power in dBm.
     */
    void UeUlTxStartCallback(double txPowerDbm);

    /**
     * @brief Called when UL transmission ends on the UE.
     *
     * Transitions UE energy model state back from NR_UE_UL_TX to
     * NR_UE_PDCCH_ONLY or NR_UE_MICRO_SLEEP depending on DRX state.
     */
    void UeUlTxEndCallback();

    Ptr<NrUePhy> m_uePhy;           //!< Attached UE PHY (may be null)
    Ptr<NrGnbPhy> m_gnbPhy;         //!< Attached gNB PHY (may be null)
    Ptr<NrUeEnergyModel> m_ueModel; //!< Attached UE energy model (may be null)
    Ptr<NrGnbEnergyModel> m_gnbModel; //!< Attached gNB energy model (may be null)

    double m_lastDlSf; //!< Last DL sf computed: allocated_RBs / total_RBs
    double m_lastUlSf; //!< Last UL sf computed
    double m_lastSp;   //!< Last sp computed: currentTxPower_lin / refTxPower_lin
    double m_lastSa;   //!< Last sa: activeTRxRUs / totalTRxRUs

    uint32_t m_totalBwpRbs;         //!< Total RBs in active BWP (from NrGnbPhy)
    double m_referenceTxPowerDbm;   //!< Reference Tx power for sp computation
};

} // namespace ns3

#endif // NR_PHY_ENERGY_LISTENER_H