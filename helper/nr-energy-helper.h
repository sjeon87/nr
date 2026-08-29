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

#include "ns3/device-energy-model-container.h"
#include "ns3/energy-source-container.h"
#include "ns3/net-device-container.h"
#include "ns3/object-factory.h"

namespace ns3
{

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
 * @warning Read the consumed energy from the returned device models
 * (GetTotalEnergyConsumption()), NOT from the energy sources. The gNB model
 * accounts energy symbol-by-symbol (TR 38.864 Section 5.2) and never drives the
 * source through the discrete ChangeState() path, so the source's own remaining
 * energy does not reflect the gNB's real consumption. The returned models are
 * the single source of truth.
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
     * @brief Install the gNB energy stack on a set of gNB devices.
     *
     * For each device i: creates an NrGnbEnergyModel, connects it to
     * sources.Get(i), and attaches an NrGnbPhyEnergyListener to the device's
     * first-BWP NrGnbPhy. @p gnbDevs and @p sources must be index-aligned and
     * of equal size (one source per node, in the same order the devices were
     * installed).
     *
     * @param gnbDevs gNB NetDevices from NrHelper::InstallGnbDevice.
     * @param sources One energy::EnergySource per gNB node, index-aligned.
     * @return Container of the created gNB device energy models.
     */
    energy::DeviceEnergyModelContainer InstallGnb(NetDeviceContainer gnbDevs,
                                                  energy::EnergySourceContainer sources);

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

    ObjectFactory m_gnbModelFactory; //!< Builds NrGnbEnergyModel instances
    ObjectFactory m_ueModelFactory;  //!< Builds NrUeEnergyModel instances
    ObjectFactory m_drxFactory;      //!< Builds NrUeDrxModel instances

    bool m_enableDrx; //!< Install a DRX model on each UE (default false)
};

} // namespace ns3

#endif // NR_ENERGY_HELPER_H
