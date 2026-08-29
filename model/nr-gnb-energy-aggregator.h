// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - multi-carrier BS power consumption

#ifndef NR_GNB_ENERGY_AGGREGATOR_H
#define NR_GNB_ENERGY_AGGREGATOR_H

#include "nr-gnb-energy-model.h"

#include "ns3/device-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/traced-value.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup nr
 * @brief Device-level multi-carrier gNB energy model (TR 38.864 Section 5.1).
 *
 * A gNB device may operate several component carriers, each modelled by its own
 * NrGnbEnergyModel. This class combines them into the single power the device
 * actually draws:
 *
 * @verbatim
     P_device(t)  =  SUM over carriers i of  w_i * P_i(t)
   @endverbatim
 *
 * per TR 38.864 Section 5.1: *"For multi-carrier, the total power consumption of
 * BS is calculated as is the sum of the power consumption of each CC; for
 * intra-band multi-carrier with contiguous CCs, the power consumption of each
 * additional CC is scaled by 0.7."*
 *
 * Each carrier therefore contributes its **complete** power, its own static
 * baseline P3 included. That is deliberate and is what the specification says:
 * the sharing of hardware between carriers is expressed by the 0.7 factor, not
 * by counting the baseline once. The contrast with the multi-TRP sentence in the
 * same clause, which does say *"if shared, is accounted once"*, confirms it.
 *
 * @note This is the opposite of how the bandwidth parts *inside* one carrier
 * combine. There the occupancies are aggregated first and the power formula is
 * evaluated once (NrGnbEnergyModel::ReportBwpOccupancy), because P_DL contains
 * the baseline P3 and the load-independent share A, so evaluating per BWP and
 * then combining would count both once per BWP. Occupancy composes inside a
 * carrier; power composes across carriers.
 *
 * @par Where the topology knowledge lives
 * This class holds nothing but a list of carriers and a weight for each. It
 * knows nothing about bands, frequencies or contiguity: classifying carriers as
 * intra-band contiguous, intra-band non-contiguous or inter-band, and deriving
 * the weights from that, is the job of NrEnergyHelper, which is the only place
 * that sees the OperationBandInfo topology.
 *
 * @par The aggregator is optional
 * NrGnbEnergyAggregator and NrGnbEnergyModel are siblings: both are
 * energy::DeviceEnergyModel, so either can be attached to an
 * energy::EnergySource. A single-carrier device attaches its NrGnbEnergyModel
 * directly and never instantiates this class.
 *
 * @warning When an aggregator is used, **only the aggregator may be appended to
 * the energy source**. energy::BasicEnergySource sums DoGetCurrentA() over every
 * appended model without weights, so appending the per-carrier models as well
 * would make the source integrate the unweighted sum while this class reports
 * the weighted one - the 0.7 would silently disappear from the source and the
 * model/source reconciliation would break.
 */
class NrGnbEnergyAggregator : public energy::DeviceEnergyModel
{
  public:
    /**
     * @brief Scaling of each additional intra-band contiguous CC.
     *
     * TR 38.864 Section 5.1, verbatim: *"for intra-band multi-carrier with
     * contiguous CCs, the power consumption of each additional CC is scaled by
     * 0.7."* A specification constant, not a tunable.
     */
    static constexpr double CONTIGUOUS_CC_SCALING = 0.7;

    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrGnbEnergyAggregator constructor
     */
    NrGnbEnergyAggregator();

    /**
     * @brief ~NrGnbEnergyAggregator
     */
    ~NrGnbEnergyAggregator() override;

    /**
     * @brief Register one component carrier by its place in the spectrum.
     *
     * The caller supplies where the carrier sits; this class decides what that
     * means for the power, because the 0.7 of TR 38.864 Section 5.1 is a rule of
     * the specification and belongs with the other formulas rather than in a
     * helper. Registering a carrier re-derives every derived weight, so the
     * order of registration does not matter.
     *
     * Two carriers are intra-band contiguous when they share a band and the
     * upper edge of one meets the lower edge of the next, within
     * ContiguityToleranceHz. The first carrier of each such run keeps 1.0 and
     * every *additional* one is scaled by 0.7. A carrier that is alone in its
     * band, or separated by a gap, is the start of a new run and keeps 1.0.
     *
     * The carrier's model must NOT be appended to the energy source; see the
     * class-level warning.
     *
     * @param model            Energy model of the carrier. Must not be null.
     * @param bandId           Operation band this carrier belongs to.
     * @param lowerFrequencyHz Lower edge of the carrier [Hz].
     * @param higherFrequencyHz Upper edge of the carrier [Hz]. Must exceed the lower edge.
     */
    void AddCarrier(Ptr<NrGnbEnergyModel> model,
                    uint8_t bandId,
                    double lowerFrequencyHz,
                    double higherFrequencyHz);

    /**
     * @brief Register a carrier with an explicit weight, bypassing the rule.
     *
     * An escape hatch for topologies the contiguity test cannot express, and for
     * unit tests that want to pin a weight directly. Such a carrier keeps the
     * weight given here and is ignored when the derived weights are recomputed,
     * so the two kinds can be mixed on one device.
     *
     * @param model  Energy model of the carrier. Must not be null.
     * @param weight Multi-carrier weight. Must be positive.
     */
    void AddCarrierWithWeight(Ptr<NrGnbEnergyModel> model, double weight);

    /**
     * @brief Number of registered carriers.
     * @return Carrier count.
     */
    uint32_t GetNCarriers() const;

    /**
     * @brief Energy model of one registered carrier, for per-carrier reporting.
     * @param index Carrier index in registration order.
     * @return The carrier's energy model.
     */
    Ptr<NrGnbEnergyModel> GetCarrier(uint32_t index) const;

    /**
     * @brief Weight applied to one registered carrier.
     * @param index Carrier index in registration order.
     * @return The weight.
     */
    double GetCarrierWeight(uint32_t index) const;

    /**
     * @brief Weighted device power drawn right now [W].
     *
     * Named to match NrGnbEnergyModel::GetInstantaneousPowerW(), so a single
     * carrier's model and a multi-carrier aggregator can be read through the
     * same call and swapped for one another.
     *
     * @return Device power in Watts.
     */
    double GetInstantaneousPowerW() const;

    /**
     * @brief Weighted device power drawn right now [W].
     *
     * Reads every carrier's current power and applies its weight. This is the
     * single quantity both GetTotalEnergyJ() and DoGetCurrentA() are built from,
     * so the attached energy source integrates exactly what this class accounts.
     *
     * @return Device power in Watts.
     */
    double ComputeDevicePowerW() const;

    /**
     * @brief Total device energy consumed so far [J].
     *
     * The committed total plus the not-yet-committed part of the interval in
     * progress. Pure: it writes nothing.
     *
     * @return Energy in Joules.
     */
    double GetTotalEnergyJ() const;

    // ----- energy::DeviceEnergyModel -----

    /**
     * @brief Attach the energy source that this device drains.
     * @param source The energy source.
     */
    void SetEnergySource(Ptr<energy::EnergySource> source) override;

    /**
     * @brief Total energy consumed, for the ns-3 energy framework.
     * @return Energy in Joules.
     */
    double GetTotalEnergyConsumption() const override;

    /**
     * @brief Forward a discrete state change to every carrier.
     *
     * A device-level state such as a future cell DTX applies to the whole gNB,
     * so it is applied to each carrier. Each carrier commits its own interval
     * and updates its own power, which this class then re-aggregates.
     *
     * @param newState New NrGnbPowerState, as int per DeviceEnergyModel.
     */
    void ChangeState(int newState) override;

    /**
     * @brief Forwarded to every carrier.
     */
    void HandleEnergyDepletion() override;

    /**
     * @brief Forwarded to every carrier.
     */
    void HandleEnergyRecharged() override;

    /**
     * @brief Forwarded to every carrier.
     */
    void HandleEnergyChanged() override;

  protected:
    /**
     * @brief Seed the device power from the carriers. Inherited from Object.
     */
    void DoInitialize() override;

    /**
     * @brief Release the carriers. Inherited from Object.
     */
    void DoDispose() override;

  private:
    /**
     * @brief Current drawn from the source [A].
     *
     * The weighted device power over the supply voltage. Deliberately the same
     * quantity GetTotalEnergyJ() integrates, so the two cannot diverge.
     *
     * @return Current in Amperes.
     */
    double DoGetCurrentA() const override;

    /**
     * @brief A carrier's power changed: close the interval and re-aggregate.
     *
     * Connected to every carrier's "InstantaneousPower" trace. The parameters
     * are unused because the new device power is read back from all carriers
     * rather than patched in place - which is also why the carrier that fired
     * does not need to be identified.
     */
    void CarrierPowerChanged(double, double);

    /**
     * @brief Commit the elapsed interval at the current device power.
     *
     * Charges the elapsed time and refreshes the energy source while the old
     * power is still installed, then advances the timestamp. Must be called
     * *before* the device power changes, so that the model and the source close
     * the same interval at the same power.
     */
    void CommitInterval();

    /// One registered component carrier.
    struct Carrier
    {
        Ptr<NrGnbEnergyModel> model; //!< The carrier's energy model
        double weight;               //!< TR 38.864 multi-carrier weight
        bool explicitWeight;         //!< True if the caller pinned the weight
        uint8_t bandId;              //!< Operation band, for the contiguity test
        double lowerFrequencyHz;     //!< Lower edge of the carrier
        double higherFrequencyHz;    //!< Upper edge of the carrier
    };

    /**
     * @brief Re-derive the weight of every spectrum-registered carrier.
     *
     * TR 38.864 Section 5.1: *"for intra-band multi-carrier with contiguous CCs,
     * the power consumption of each additional CC is scaled by 0.7."* The rule is
     * about runs of touching carriers within one band, so it can only be applied
     * to the set as a whole - adding a carrier can turn a previous anchor into an
     * additional CC of a longer run. That is why this recomputes all of them
     * rather than assigning a weight at registration, and why the answer does not
     * depend on the order carriers were added in.
     *
     * Carriers registered through AddCarrierWithWeight() are left alone.
     */
    void RecomputeWeights();

    /**
     * @brief Common tail of both AddCarrier forms.
     *
     * Commits the open interval, stores the carrier, subscribes to its power
     * trace, re-derives the weights and installs the new device power.
     *
     * @param carrier The carrier to register.
     */
    void RegisterCarrier(const Carrier& carrier);

    Ptr<energy::EnergySource> m_source; //!< Attached energy source (may be null)
    std::vector<Carrier> m_carriers;    //!< Registered carriers, in order

    double m_contiguityToleranceHz; //!< Edge-matching slack of the contiguity test

    double m_currentPowerW; //!< Weighted device power installed for the interval in progress
    Time m_lastUpdateTime;  //!< Start of that interval

    TracedValue<double> m_powerTrace;   //!< Device power [W], fires on every change
    TracedValue<double> m_totalEnergyJ; //!< Committed energy [J], excl. the open interval
};

} // namespace ns3

#endif // NR_GNB_ENERGY_AGGREGATOR_H
