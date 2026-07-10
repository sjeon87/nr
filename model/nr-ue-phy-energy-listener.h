// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#ifndef NR_UE_PHY_ENERGY_LISTENER_H
#define NR_UE_PHY_ENERGY_LISTENER_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/sfnsf.h"
namespace ns3
{

// Forward declarations
class NrUePhy;
class NrUeEnergyModel;


/**
 * @ingroup nr
 * @brief Callback bridge between NR UE PHY layer events and the UE energy model.
 *
 * This class is the decoupling layer between the UE's PHY event stream and the
 * UE energy model state machine. It subscribes to trace sources on NrUePhy,
 * extracts the state transitions required by the 3GPP TR 38.840 UE power
 * states, and forwards energy model state-change calls to NrUeEnergyModel.
 *
 * Design rationale:
 *   - This class must NOT be aware of the energy formula internals.
 *     It only knows: "event occurred, extract parameters, notify model."
 *   - It must be usable both with and without an energy model installed
 *     (if no model is attached, callbacks are no-ops).
 *   - It has a single responsibility: the UE side of the mapping. The gNB side
 *     lives in a separate listener.
 *
 */
class NrUePhyEnergyListener : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrUePhyEnergyListener constructor
     */
    NrUePhyEnergyListener();
    /**
     * @brief ~NrUePhyEnergyListener
     */
    ~NrUePhyEnergyListener() override;

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
    void SetPhy(Ptr<NrUePhy> phy);

    /**
     * @brief Connect the UE energy model that this listener will drive.
     * @param model Pointer to the NrUeEnergyModel instance.
     */
    void SetEnergyModel(Ptr<NrUeEnergyModel> model);

  protected:
    /**
     * @brief Release attached pointers. Inherited from Object.
     */
    void DoDispose() override;

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
    void SlotIndicationCallback(const SfnSf& sfnSf);

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
    void PdcchDecodeSuccessCallback(bool hasDownlinkGrant, bool hasUplinkGrant);

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
    void UlTxStartCallback(double txPowerDbm);

    /**
     * @brief Called when UL transmission ends on the UE.
     *
     * Transitions UE energy model state back from NR_UE_UL_TX to
     * NR_UE_PDCCH_ONLY or NR_UE_MICRO_SLEEP depending on DRX state.
     */
    void UlTxEndCallback();

    Ptr<NrUePhy> m_phy;             //!< Attached UE PHY (may be null)
    Ptr<NrUeEnergyModel> m_model;   //!< Attached UE energy model (may be null)
};

} // namespace ns3

#endif // NR_UE_PHY_ENERGY_LISTENER_H
