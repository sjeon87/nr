// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5 - Energy consumption model for BS
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#ifndef NR_ENERGY_HELPER_H
#define NR_ENERGY_HELPER_H

#include "cc-bwp-helper.h"

#include "ns3/device-energy-model-container.h"
#include "ns3/energy-source-container.h"
#include "ns3/net-device-container.h"
#include "ns3/object-factory.h"

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

class NrGnbEnergyModel;

/**
 * @ingroup nr
 * @brief One-call installer for the NR energy framework.
 *
 * Wires the NR energy stack onto already built NR devices, so a user scenario
 * never has to touch the individual models or the PHY-to-model callback
 * bridges. For each device it creates the per-device energy model, connects it
 * to the caller's energy::EnergySource, and attaches the matching PHY listener
 * that translates PHY events into TR 38.864 / TR 38.840 power-state changes:
 *   - gNB: NrGnbEnergyModel + NrGnbPhyEnergyListener (TR 38.864 Section 5);
 *   - UE:  NrUeEnergyModel  + NrUePhyEnergyListener  (TR 38.840 Section 8), and
 *          an optional NrUeDrxModel when EnableDrx() is set.
 *
 * The energy sources themselves are NOT created here: they are standard ns-3
 * energy::EnergySource objects the caller installs with e.g.
 * BasicEnergySourceHelper, one per node, passed in index aligned with the
 * device container. The created device models are returned in a
 * DeviceEnergyModelContainer so the caller can read GetTotalEnergyConsumption()
 * afterwards; the listeners and DRX models are aggregated onto their node so
 * they outlive this (typically stack-allocated) helper.
 *
 * The returned models and their energy sources agree: each model refreshes its
 * source over the same intervals it charges internally, so
 * GetTotalEnergyConsumption() and the source's drain report the same energy.
 * Either may be read.
 *
 * Typical use (see the cttc-nr-energy examples):
 * @code
 *   NrEnergyHelper energyHelper;
 *   energyHelper.EnableDrx(true); // optional
 *   auto gnbModels = energyHelper.InstallGnb(gnbDevs, gnbSources);
 *   auto ueModels  = energyHelper.InstallUe(ueDevs, ueSources);
 * @endcode
 */
class NrEnergyHelper
{
  public:
    /**
     * @brief NrEnergyHelper constructor.
     *
     * Configures the internal object factories with the default NR energy
     * model / DRX TypeIds; the 3GPP defaults on those models apply until
     * overridden with the SetXxxAttribute() methods below.
     */
    NrEnergyHelper();

    /**
     * @brief ~NrEnergyHelper
     */
    ~NrEnergyHelper();

    // ----- Model configuration (optional; sensible 3GPP defaults otherwise) -----

    /**
     * @brief Set an attribute on every gNB energy model created afterwards.
     * @param n Attribute name on NrGnbEnergyModel.
     * @param v Attribute value.
     */
    void SetGnbEnergyModelAttribute(std::string n, const AttributeValue& v);

    /**
     * @brief Set an attribute on the energy model of ONE component carrier.
     *
     * Carriers of a device need not share a 3GPP configuration. TR 38.864
     * Table 5.1-1 defines separate reference sets for FR1 and FR2, so a device
     * whose carriers span both ranges needs a different RefConfigSet, and with
     * it a different P1..P5 row, on each. Anything set here applies to that
     * carrier only and overrides SetGnbEnergyModelAttribute() for it.
     *
     * Carrier indices follow the order the carriers appear in the operation
     * bands passed to InstallGnb(), which is also the order
     * NrGnbEnergyAggregator::GetCarrier() reports them in.
     *
     * @code
     *   // carrier 0 on FR1, carrier 1 on FR2
     *   helper.SetGnbEnergyModelAttributeForCc(0, "RefConfigSet", EnumValue(Set1));
     *   helper.SetGnbEnergyModelAttributeForCc(1, "RefConfigSet", EnumValue(Set3));
     * @endcode
     *
     * @param ccIndex Component carrier index.
     * @param n Attribute name on NrGnbEnergyModel.
     * @param v Attribute value.
     */
    void SetGnbEnergyModelAttributeForCc(uint32_t ccIndex, std::string n, const AttributeValue& v);

    /**
     * @brief Set an attribute on every UE energy model created afterwards.
     * @param n Attribute name on NrUeEnergyModel.
     * @param v Attribute value.
     */
    void SetUeEnergyModelAttribute(std::string n, const AttributeValue& v);

    /**
     * @brief Set an attribute on every DRX model created afterwards.
     * @param n Attribute name on NrUeDrxModel.
     * @param v Attribute value.
     */
    void SetDrxModelAttribute(std::string n, const AttributeValue& v);

    /**
     * @brief Enable or disable C-DRX on the UEs installed afterwards.
     *
     * When enabled, InstallUe() also creates an NrUeDrxModel per UE, connects
     * it to the UE energy model and the PHY listener, and starts its cycling.
     * When disabled (the default) the UE stays in PDCCH-only monitoring between
     * transport blocks with no sleep cycling.
     *
     * @param enable True to install a DRX model on each UE.
     */
    void EnableDrx(bool enable);

    // ----- Installation -----

    /**
     * @brief Install the gNB energy stack, treating each device as ONE carrier.
     *
     * For each device i: creates one NrGnbEnergyModel, connects it to
     * sources.Get(i), and attaches an NrGnbPhyEnergyListener to *every* bandwidth
     * part of the device. All of them report into that single model, which
     * aggregates their occupancy and evaluates the TR 38.864 formula once - the
     * right thing for a device whose bandwidth parts belong to one component
     * carrier, and the common case.
     *
     * @p gnbDevs and @p sources must be index-aligned and of equal size (one
     * source per node, in the same order the devices were installed).
     *
     * @note A device carrying several component CARRIERS needs the overload
     * below. Without the band topology this class cannot tell two bandwidth
     * parts of one carrier from two carriers, and the two are charged
     * differently: occupancy composes inside a carrier, power composes across
     * carriers with 0.7 on each additional intra-band contiguous one.
     *
     * @param gnbDevs gNB NetDevices from NrHelper::InstallGnbDevice.
     * @param sources One energy::EnergySource per gNB node, index-aligned.
     * @return Container of the created gNB device energy models, one per device.
     */
    energy::DeviceEnergyModelContainer InstallGnb(NetDeviceContainer gnbDevs,
                                                  energy::EnergySourceContainer sources);

    /**
     * @brief Install the gNB energy stack for a multi-carrier device.
     *
     * Pass the same operation bands that were given to CcBwpCreator::GetAllBwps()
     * to build the devices. That flattening - bands in order, then component
     * carriers, then bandwidth parts - is what fixes the bandwidth part indices
     * on the device, so replaying it here recovers which BWP belongs to which
     * carrier. The device itself cannot answer that: its "CC map" is keyed by
     * bandwidth part, and the carrier grouping exists only in OperationBandInfo.
     *
     * For each device this creates one NrGnbEnergyModel **per component
     * carrier**, attaches a listener to each bandwidth part pointing at its
     * carrier's model, and then:
     *   - one carrier  -> that model is attached to the energy source directly,
     *     which is exactly what the single-carrier overload produces;
     *   - several      -> an NrGnbEnergyAggregator is created, each carrier is
     *     registered with its band and frequency edges so the aggregator derives
     *     the TR 38.864 Section 5.1 weights itself, and **only the aggregator**
     *     is appended to the source.
     *
     * @param gnbDevs gNB NetDevices from NrHelper::InstallGnbDevice.
     * @param sources One energy::EnergySource per gNB node, index-aligned.
     * @param bands   The operation bands the devices were built from, in the same
     *                order they were passed to CcBwpCreator::GetAllBwps().
     * @return Container of the created device-level models: the aggregator for a
     *         multi-carrier device, the carrier's model for a single-carrier one.
     *         Per-carrier models remain reachable through
     *         NrGnbEnergyAggregator::GetCarrier().
     */
    energy::DeviceEnergyModelContainer InstallGnb(
        NetDeviceContainer gnbDevs,
        energy::EnergySourceContainer sources,
        const std::vector<std::reference_wrapper<OperationBandInfo>>& bands);

    /**
     * @brief Install the UE energy stack on a set of UE devices.
     *
     * For each device i: creates an NrUeEnergyModel, connects it to
     * sources.Get(i), attaches an NrUePhyEnergyListener to the device's
     * first-BWP NrUePhy, and (if EnableDrx() was set) an NrUeDrxModel. @p ueDevs
     * and @p sources must be index-aligned and of equal size.
     *
     * @param ueDevs  UE NetDevices from NrHelper::InstallUeDevice.
     * @param sources One energy::EnergySource per UE node, index-aligned.
     * @return Container of the created UE device energy models.
     */
    energy::DeviceEnergyModelContainer InstallUe(NetDeviceContainer ueDevs,
                                                 energy::EnergySourceContainer sources);

    /**
     * @brief Install the UE energy stack on a set of UE devices, listening on
     *        a caller-chosen bandwidth part.
     *
     * For each device i: creates an NrUeEnergyModel, connects it to
     * sources.Get(i), attaches an NrUePhyEnergyListener to the device's PHY
     * for @p bwpIndex, and (if EnableDrx() was set) an NrUeDrxModel. @p ueDevs
     * and @p sources must be index-aligned and of equal size. @p bwpIndex must
     * exist on every device in @p ueDevs.
     *
     * @param ueDevs   UE NetDevices from NrHelper::InstallUeDevice.
     * @param sources  One energy::EnergySource per UE node, index-aligned.
     * @param bwpIndex Bandwidth part to attach the energy listener to.
     * @return Container of the created UE device energy models.
     */
    energy::DeviceEnergyModelContainer InstallUe(NetDeviceContainer ueDevs,
                                                 energy::EnergySourceContainer sources,
                                                 uint32_t bwpIndex);

    /**
     * @brief Install both sides in one call.
     *
     * Convenience wrapper around InstallGnb() and InstallUe(); the returned
     * container concatenates the gNB models followed by the UE models.
     *
     * @param gnbDevs    gNB NetDevices.
     * @param gnbSources One energy::EnergySource per gNB node, index-aligned.
     * @param ueDevs     UE NetDevices.
     * @param ueSources  One energy::EnergySource per UE node, index-aligned.
     * @return Container of all created device energy models (gNB then UE).
     */
    energy::DeviceEnergyModelContainer Install(NetDeviceContainer gnbDevs,
                                               energy::EnergySourceContainer gnbSources,
                                               NetDeviceContainer ueDevs,
                                               energy::EnergySourceContainer ueSources);

  private:
    /**
     * @brief Connect a device energy model to an energy source both ways.
     * @param model  The device energy model to attach.
     * @param source The energy source that drains it.
     */
    void AttachToSource(Ptr<energy::DeviceEnergyModel> model, Ptr<energy::EnergySource> source);

    /// Which component carrier a bandwidth part belongs to, and where it sits.
    struct BwpToCarrier
    {
        uint32_t carrierKey;      //!< Unique per (band, CC); groups the BWPs
        uint8_t bandId;           //!< Operation band of that carrier
        double lowerFrequencyHz;  //!< Lower edge of the carrier
        double higherFrequencyHz; //!< Upper edge of the carrier
    };

    /**
     * @brief Replay the CcBwpCreator flattening to map BWP index -> carrier.
     *
     * CcBwpCreator::GetAllBwps() concatenates the bands in order, and
     * OperationBandInfo::GetBwps() walks that band's carriers in order and each
     * carrier's bandwidth parts in order. Indexing the result the same way
     * reproduces the bandwidth part numbering the devices were built with.
     *
     * @param bands The operation bands, in the order given to GetAllBwps().
     * @return One entry per bandwidth part, indexed by its device BWP index.
     */
    static std::vector<BwpToCarrier> MapBwpsToCarriers(
        const std::vector<std::reference_wrapper<OperationBandInfo>>& bands);

    /**
     * @brief Attach a listener to one bandwidth part's PHY.
     *
     * The listener is aggregated onto the **NrGnbPhy**, not onto the node: a
     * node accepts only one object of a given TypeId, so a second listener there
     * aborts the simulation as soon as a device has two bandwidth parts. There
     * is exactly one listener per PHY, and the PHY lives as long as the device.
     *
     * @param dev      The gNB device.
     * @param bwpIndex Bandwidth part index on that device.
     * @param model    The carrier energy model this bandwidth part reports into.
     */
    void AttachGnbListener(Ptr<NetDevice> dev, uint32_t bwpIndex, Ptr<NrGnbEnergyModel> model);

    /**
     * @brief Number of bandwidth parts on a gNB device.
     *
     * NrGnbNetDevice's "CC map" is keyed by bandwidth part rather than by
     * component carrier, so its size is the bandwidth part count. Which of them
     * share a carrier is not recorded on the device at all - only
     * OperationBandInfo knows that.
     *
     * @param dev The gNB device.
     * @return Bandwidth part count, at least 1.
     */
    static uint32_t GetGnbBwpCount(Ptr<NetDevice> dev);

    /**
     * @brief Build the model for one carrier, applying any per-carrier overrides.
     * @param ccIndex Component carrier index.
     * @return A configured NrGnbEnergyModel.
     */
    Ptr<NrGnbEnergyModel> CreateCarrierModel(uint32_t ccIndex);

    ObjectFactory m_gnbModelFactory; //!< Builds NrGnbEnergyModel instances
    /// Per-carrier attribute overrides, applied on top of m_gnbModelFactory.
    std::map<uint32_t, std::vector<std::pair<std::string, Ptr<AttributeValue>>>> m_ccAttributes;
    ObjectFactory m_ueModelFactory; //!< Builds NrUeEnergyModel instances
    ObjectFactory m_drxFactory;     //!< Builds NrUeDrxModel instances

    bool m_enableDrx; //!< Install a DRX model on each UE (default false)
};

} // namespace ns3

#endif // NR_ENERGY_HELPER_H
