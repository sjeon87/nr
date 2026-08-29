// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#ifndef NR_UE_PHY_ENERGY_LISTENER_H
#define NR_UE_PHY_ENERGY_LISTENER_H

#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <algorithm>

namespace ns3
{

// Forward declarations
class NrUePhy;
class NrUeEnergyModel;
class NrUeDrxModel;
class NrUePhyEnergyListenerSlotAveragedDurationTestCase;
class NrUePhyEnergyListenerRankDefaultOffTestCase;
class NrUePhyEnergyListenerRankOptInTestCase;

/**
 * @ingroup nr
 * @brief Callback bridge between NR UE PHY layer events and the UE energy model.
 *
 * This class is the decoupling layer between the UE's PHY event stream and the
 * UE energy model state machine. It subscribes to existing trace sources on
 * NrUePhy and drives the TR 38.840 Section 8.1 UE power states:
 *   ReportDownlinkTbSize - DL TB delivered -> NR_UE_PDCCH_PDSCH
 *   ReportUplinkTbSize   - UL TB sent      -> NR_UE_UL_TX
 * One slot after an active DL/UL slot the UE returns to PDCCH-only
 * monitoring. Sleep transitions are owned by the NrUeDrxModel,
 * whose inactivity timer this listener restarts on each transport block.
 *
 * Active-state power scaling is BWP-bandwidth based (TR 38.840 Section
 * 8.1.3): on each transport block the listener re-reads the PHY channel
 * bandwidth and re-applies NrUeEnergyModel::ApplyBwpScaling() when it
 * changed, so BWP switches are reflected in the UE power draw.
 *
 * The TR 38.840 Table 21 receive-chain scaling is NOT driven from here by
 * default: 5G-LENA has no receive-chain adaptation, so the powered-chain count
 * is the static NrUeEnergyModel::ActiveRxChains configuration. UseRankAsRxChains
 * enables an optional, explicitly non-3GPP mapping of MIMO rank onto it instead.
 *
 * Design rationale:
 *   - This class must NOT be aware of the energy formula internals.
 *     It only knows: event occurred, extract parameters, notify model.
 *   - It must be usable both with and without an energy model installed
 *     (if no model is attached, callbacks are no-ops).
 *   - It has a single responsibility: the UE side of the mapping. The gNB side
 *     lives in a separate listener.
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
     *   - ReportDownlinkTbSize - DL transport block delivered
     *   - ReportUplinkTbSize   - UL transport block sent
     * Also caches the slot period for the return-to-monitoring timer and
     * applies the initial BWP scaling from the PHY channel bandwidth.
     *
     * @param phy Pointer to the NrUePhy on the UE node.
     */
    void SetPhy(Ptr<NrUePhy> phy);

    /**
     * @brief Connect the UE energy model that this listener will drive.
     * @param model Pointer to the NrUeEnergyModel instance.
     */
    void SetEnergyModel(Ptr<NrUeEnergyModel> model);

    /**
     * @brief Connect a DRX model to be notified of UE data activity.
     *
     * When set, each DL/UL transport block restarts the DRX inactivity timer
     * via NrUeDrxModel::NotifyDataActivity(), so the DRX model owns the sleep
     * transitions while this listener owns the active reception/transmission
     * states.
     *
     * @param drx Pointer to the NrUeDrxModel instance.
     */
    void SetDrxModel(Ptr<NrUeDrxModel> drx);

  protected:
    /**
     * @brief Release attached pointers. Inherited from Object.
     */
    void DoDispose() override;

  private:
    // Unit tests drive the private PHY callbacks directly.
    friend class NrUePhyEnergyListenerSlotAveragedDurationTestCase;
    friend class NrUePhyEnergyListenerRankDefaultOffTestCase;
    friend class NrUePhyEnergyListenerRankOptInTestCase;

    /**
     * @brief UE DL driver, connected to NrUePhy "ReportDownlinkTbSize".
     *
     * Refreshes the BWP scaling, moves the UE energy model to
     * NR_UE_PDCCH_PDSCH for the active slot and schedules a return to
     * NR_UE_PDCCH_ONLY one slot later (TR 38.840 Table 18). When the UE wakes
     * from deep sleep, the configured setup transient is charged first.
     *
     * @param imsi     UE IMSI.
     * @param tbSize   Downlink transport block size in bytes (0 = nothing).
     * @param symStart First OFDM symbol of the DL allocation.
     * @param numSym   OFDM symbols in the DL allocation. Not used for the active
     *                 duration: Table 18/20 powers are already slot-averaged.
     * @param rank     Number of MIMO layers in use.
     */
    void DlTbReceivedCallback(uint64_t imsi,
                              uint32_t tbSize,
                              uint32_t symStart,
                              uint32_t numSym,
                              uint32_t rank);

    /**
     * @brief UE UL driver, connected to NrUePhy "ReportUplinkTbSize".
     *
     * Selects the UL relative power from the current PHY Tx power
     * (TR 38.840 Table 18: 250 units at 0 dBm to 700 units at 23 dBm), moves
     * the UE energy model to NR_UE_UL_TX and schedules a return to
     * NR_UE_PDCCH_ONLY one slot later.
     *
     * @param imsi     UE IMSI.
     * @param tbSize   Uplink transport block size in bytes (0 = nothing).
     * @param symStart First OFDM symbol of the UL allocation.
     * @param numSym   OFDM symbols in the UL allocation. Not used for the active
     *                 duration: Table 18/20 powers are already slot-averaged.
     * @param rank     Number of MIMO layers in use.
     */
    void UlTbSentCallback(uint64_t imsi,
                          uint32_t tbSize,
                          uint32_t symStart,
                          uint32_t numSym,
                          uint32_t rank);

    /**
     * @brief Return the UE energy model to PDCCH-only monitoring.
     *
     * Scheduled one slot after an active DL/UL slot.
     */
    void ReturnToMonitoring();

    /**
     * @brief Re-apply the BWP scaling if the active BWP bandwidth changed.
     *
     * Reads the PHY channel bandwidth and calls
     * NrUeEnergyModel::ApplyBwpScaling() when it differs from the last applied
     * value (TR 38.840 Section 8.1.3). This is how BWP switches reach the
     * energy model without any PHY-side hook.
     */
    void RefreshBwpScaling();

    Ptr<NrUePhy> m_phy;           //!< Attached UE PHY (may be null)
    Ptr<NrUeEnergyModel> m_model; //!< Attached UE energy model (may be null)
    Ptr<NrUeDrxModel> m_drx;      //!< Attached DRX model (may be null)

    Time m_slotDuration; //!< Cached slot period (for the return timer)
    Time m_activeUntil{Seconds(0)};
    uint32_t m_lastBwpMhz;    //!< Last BWP bandwidth applied to the model [MHz]
    bool m_useRankAsRxChains; //!< Opt-in: map MIMO rank onto the Table 21 scaling
};

} // namespace ns3

#endif // NR_UE_PHY_ENERGY_LISTENER_H
