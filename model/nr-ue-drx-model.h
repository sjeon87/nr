// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE power saving / C-DRX evaluation
//   TS 38.321: MAC C-DRX procedure (onDuration, inactivity, long cycle)

#ifndef NR_UE_DRX_MODEL_H
#define NR_UE_DRX_MODEL_H

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

namespace ns3
{

class NrUeEnergyModel;

/**
 * @ingroup nr
 * @brief Connected-mode DRX power model that drives the UE energy state machine.
 *
 * 5G-LENA has no MAC C-DRX engine, so for TR 38.840 power-saving evaluation
 * this class runs the DRX timers (onDuration, inactivity, long cycle) on the
 * ns-3 scheduler and toggles the attached NrUeEnergyModel between active
 * monitoring (PDCCH-only) and sleep (light / deep) accordingly. It is a power
 * model for energy evaluation, not a protocol-accurate MAC DRX: it does not
 * gate the scheduler, it only decides the UE power state over time.
 *
 * Behaviour (TS 38.321 semantics):
 *   - Each long cycle the onDuration starts: UE wakes to PDCCH-only monitoring.
 *   - Data activity (NotifyDataActivity, fired by NrUePhyEnergyListener on a
 *     transport block) (re)starts the inactivity timer and keeps the UE awake.
 *   - When the onDuration ends with no running inactivity timer, or when the
 *     inactivity timer expires, the UE sleeps until the next onDuration.
 *   - Sleep depth is chosen by the gap to the next onDuration: a gap at least
 *     DeepSleepThreshold long uses deep sleep, otherwise light sleep
 *     (TR 38.840 Section 8.1: deeper sleep amortises its transition cost only
 *     over long inactive periods).
 */
class NrUeDrxModel : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrUeDrxModel constructor
     */
    NrUeDrxModel();

    /**
     * @brief ~NrUeDrxModel
     */
    ~NrUeDrxModel() override;

    /**
     * @brief Set the UE energy model that this DRX model drives.
     * @param model Pointer to the NrUeEnergyModel instance.
     */
    void SetEnergyModel(Ptr<NrUeEnergyModel> model);

    /**
     * @brief Begin DRX cycling at the given time.
     * @param startAt Absolute time of the first onDuration start.
     */
    void Start(Time startAt);

    /**
     * @brief Notify the model that data was sent/received for this UE.
     *
     * (Re)starts the inactivity timer and keeps the UE awake. Called by
     * NrUePhyEnergyListener on each DL/UL transport block.
     */
    void NotifyDataActivity();

  protected:
    void DoDispose() override;

  private:
    /**
     * @brief Start of a DRX long cycle: wake to onDuration monitoring.
     */
    void StartCycle();

    /**
     * @brief End of the onDuration window: sleep if no inactivity timer runs.
     */
    void EndOnDuration();

    /**
     * @brief Inactivity timer expiry: sleep until the next onDuration.
     */
    void InactivityExpired();

    /**
     * @brief Put the UE into the appropriate sleep state for the gap remaining.
     */
    void GoToSleep();

    Ptr<NrUeEnergyModel> m_energyModel; //!< Driven UE energy model (may be null)

    Time m_longCycle;          //!< DRX long cycle period
    Time m_onDuration;         //!< onDuration window length
    Time m_inactivityTimer;    //!< drx-InactivityTimer length
    Time m_deepSleepThreshold; //!< Gap >= this -> deep sleep, else light sleep

    Time m_nextCycleTime;     //!< Absolute time of the next onDuration start
    bool m_inactivityRunning; //!< True while the inactivity timer is armed

    EventId m_cycleEvent;      //!< Next StartCycle event
    EventId m_onDurationEvent; //!< EndOnDuration event
    EventId m_inactivityEvent; //!< InactivityExpired event
};

} // namespace ns3

#endif // NR_UE_DRX_MODEL_H
