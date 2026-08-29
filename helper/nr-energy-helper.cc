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

#include "nr-energy-helper.h"

#include "nr-helper.h"

#include "ns3/energy-source.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/nr-gnb-energy-aggregator.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy-energy-listener.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-ue-drx-model.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy-energy-listener.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrEnergyHelper");

namespace
{
// The UE listener drives the first BWP, matching the GetUePhy(dev, 0)
// convention used throughout the NR examples. The gNB side no longer has a
// single index: it wires every bandwidth part of the device.
constexpr uint32_t ENERGY_BWP_INDEX = 0;
} // namespace

NrEnergyHelper::NrEnergyHelper()
    : m_enableDrx(false)
{
    NS_LOG_FUNCTION(this);
    m_gnbModelFactory.SetTypeId(NrGnbEnergyModel::GetTypeId());
    m_ueModelFactory.SetTypeId(NrUeEnergyModel::GetTypeId());
    m_drxFactory.SetTypeId(NrUeDrxModel::GetTypeId());
}

NrEnergyHelper::~NrEnergyHelper()
{
    NS_LOG_FUNCTION(this);
}

void
NrEnergyHelper::SetGnbEnergyModelAttribute(std::string n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << n);
    m_gnbModelFactory.Set(n, v);
}

void
NrEnergyHelper::SetGnbEnergyModelAttributeForCc(uint32_t ccIndex,
                                                std::string n,
                                                const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << ccIndex << n);
    // Copy the value: the caller's temporary is gone by the time the carrier is
    // built, which happens inside InstallGnb().
    m_ccAttributes[ccIndex].emplace_back(n, v.Copy());
}

Ptr<NrGnbEnergyModel>
NrEnergyHelper::CreateCarrierModel(uint32_t ccIndex)
{
    NS_LOG_FUNCTION(this << ccIndex);
    Ptr<NrGnbEnergyModel> model = m_gnbModelFactory.Create<NrGnbEnergyModel>();

    // Per-carrier overrides last, so they win over the device-wide defaults.
    // Carriers in different frequency ranges need different TR 38.864
    // Table 5.1-1 reference sets, and so different P1..P5 rows.
    const auto it = m_ccAttributes.find(ccIndex);
    if (it != m_ccAttributes.end())
    {
        for (const auto& [name, value] : it->second)
        {
            model->SetAttribute(name, *value);
        }
    }
    return model;
}

void
NrEnergyHelper::SetUeEnergyModelAttribute(std::string n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << n);
    m_ueModelFactory.Set(n, v);
}

void
NrEnergyHelper::SetDrxModelAttribute(std::string n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << n);
    m_drxFactory.Set(n, v);
}

void
NrEnergyHelper::EnableDrx(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableDrx = enable;
}

void
NrEnergyHelper::AttachToSource(Ptr<energy::DeviceEnergyModel> model,
                               Ptr<energy::EnergySource> source)
{
    NS_LOG_FUNCTION(this << model << source);
    NS_ASSERT_MSG(source, "Every device needs an energy source");
    source->AppendDeviceEnergyModel(model);
    model->SetEnergySource(source);
}

uint32_t
NrEnergyHelper::GetGnbBwpCount(Ptr<NetDevice> dev)
{
    Ptr<NrGnbNetDevice> nrDev = DynamicCast<NrGnbNetDevice>(dev);
    NS_ABORT_MSG_IF(!nrDev, "NrEnergyHelper::InstallGnb needs NrGnbNetDevice objects");
    return std::max(1u, nrDev->GetCcMapSize());
}

void
NrEnergyHelper::AttachGnbListener(Ptr<NetDevice> dev,
                                  uint32_t bwpIndex,
                                  Ptr<NrGnbEnergyModel> model)
{
    NS_LOG_FUNCTION(this << bwpIndex);
    Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(dev, bwpIndex);
    NS_ASSERT_MSG(phy, "gNB device has no PHY for bandwidth part " << bwpIndex);

    // SetEnergyModel() before SetPhy(): the listener pushes the PHY's symbol
    // duration into the model as soon as it attaches.
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    listener->SetEnergyModel(model);
    listener->SetPhy(phy);

    // Aggregate onto the PHY rather than the node. A node holds at most one
    // object of a given TypeId, so a second listener aggregated there aborts the
    // simulation the moment a device has two bandwidth parts. One listener per
    // PHY collides with nothing, and the PHY outlives this (stack-allocated)
    // helper just as the node would.
    phy->AggregateObject(listener);
}

std::vector<NrEnergyHelper::BwpToCarrier>
NrEnergyHelper::MapBwpsToCarriers(
    const std::vector<std::reference_wrapper<OperationBandInfo>>& bands)
{
    // Reproduce the CcBwpCreator::GetAllBwps() flattening exactly: bands in the
    // order given, then each band's component carriers in order, then each
    // carrier's bandwidth parts in order. The position in that walk IS the
    // bandwidth part index the device was built with.
    std::vector<BwpToCarrier> map;
    uint32_t carrierKey = 0;

    for (const auto& bandRef : bands)
    {
        const OperationBandInfo& band = bandRef.get();
        for (const auto& cc : band.m_cc)
        {
            for (size_t bwp = 0; bwp < cc->m_bwp.size(); ++bwp)
            {
                map.push_back(
                    {carrierKey, band.m_bandId, cc->m_lowerFrequency, cc->m_higherFrequency});
            }
            ++carrierKey;
        }
    }
    return map;
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::InstallGnb(NetDeviceContainer gnbDevs, energy::EnergySourceContainer sources)
{
    NS_LOG_FUNCTION(this << gnbDevs.GetN());
    NS_ASSERT_MSG(gnbDevs.GetN() == sources.GetN(),
                  "One energy source per gNB device is required (index-aligned)");

    energy::DeviceEnergyModelContainer models;
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = gnbDevs.Get(i);
        Ptr<energy::EnergySource> source = sources.Get(i);
        NS_ASSERT_MSG(source->GetNode() == dev->GetNode(),
                      "Energy source " << i << " is not on the same node as gNB device " << i
                                       << "; the two containers must be index-aligned");

        Ptr<NrGnbEnergyModel> model = CreateCarrierModel(0);
        AttachToSource(model, source);

        // Every bandwidth part of the device reports into this one model, which
        // aggregates their occupancy and applies the formula once. Treating the
        // device as a single carrier is what makes that correct; a device with
        // several carriers must use the band-aware overload.
        const uint32_t nBwps = GetGnbBwpCount(dev);
        for (uint32_t bwp = 0; bwp < nBwps; ++bwp)
        {
            AttachGnbListener(dev, bwp, model);
        }

        models.Add(model);
    }
    return models;
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::InstallGnb(NetDeviceContainer gnbDevs,
                           energy::EnergySourceContainer sources,
                           const std::vector<std::reference_wrapper<OperationBandInfo>>& bands)
{
    NS_LOG_FUNCTION(this << gnbDevs.GetN() << bands.size());
    NS_ASSERT_MSG(gnbDevs.GetN() == sources.GetN(),
                  "One energy source per gNB device is required (index-aligned)");

    const std::vector<BwpToCarrier> bwpMap = MapBwpsToCarriers(bands);
    NS_ABORT_MSG_IF(bwpMap.empty(), "The operation bands describe no bandwidth part at all");

    energy::DeviceEnergyModelContainer models;
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = gnbDevs.Get(i);
        Ptr<energy::EnergySource> source = sources.Get(i);
        NS_ASSERT_MSG(source->GetNode() == dev->GetNode(),
                      "Energy source " << i << " is not on the same node as gNB device " << i
                                       << "; the two containers must be index-aligned");

        const uint32_t nBwps = GetGnbBwpCount(dev);
        NS_ABORT_MSG_IF(nBwps > bwpMap.size(),
                        "gNB device " << i << " has " << nBwps
                                      << " bandwidth parts but the operation bands passed here "
                                         "describe only "
                                      << bwpMap.size()
                                      << ". Pass the same bands that were given to "
                                         "CcBwpCreator::GetAllBwps() when the devices were built.");

        // One model per component carrier, created on first sight of a BWP that
        // belongs to it. Insertion order follows the BWP order, so the carriers
        // come out in frequency order for a normally built band.
        std::map<uint32_t, Ptr<NrGnbEnergyModel>> byCarrier;
        std::vector<uint32_t> carrierOrder;

        for (uint32_t bwp = 0; bwp < nBwps; ++bwp)
        {
            const uint32_t key = bwpMap[bwp].carrierKey;
            auto it = byCarrier.find(key);
            if (it == byCarrier.end())
            {
                it = byCarrier.emplace(key, CreateCarrierModel(carrierOrder.size())).first;
                carrierOrder.push_back(key);
            }
            AttachGnbListener(dev, bwp, it->second);
        }

        if (carrierOrder.size() == 1)
        {
            // Single carrier: no aggregator, so the result is bit for bit what
            // the other overload produces.
            Ptr<NrGnbEnergyModel> only = byCarrier.at(carrierOrder.front());
            AttachToSource(only, source);
            models.Add(only);
            continue;
        }

        // Several carriers: the aggregator is the device model. It derives the
        // TR 38.864 Section 5.1 weights from the band and the frequency edges,
        // so nothing here decides what is contiguous.
        Ptr<NrGnbEnergyAggregator> aggregator = CreateObject<NrGnbEnergyAggregator>();
        for (const uint32_t key : carrierOrder)
        {
            const auto& info =
                *std::find_if(bwpMap.begin(), bwpMap.end(), [key](const BwpToCarrier& b) {
                    return b.carrierKey == key;
                });
            aggregator->AddCarrier(byCarrier.at(key),
                                   info.bandId,
                                   info.lowerFrequencyHz,
                                   info.higherFrequencyHz);
        }

        // ONLY the aggregator is appended: an EnergySource sums DoGetCurrentA()
        // over its models without weights, so appending the per-carrier models
        // as well would drain the unweighted sum and lose the 0.7.
        AttachToSource(aggregator, source);
        models.Add(aggregator);
    }
    return models;
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::InstallUe(NetDeviceContainer ueDevs, energy::EnergySourceContainer sources)
{
    return InstallUe(ueDevs, sources, ENERGY_BWP_INDEX);
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::InstallUe(NetDeviceContainer ueDevs,
                          energy::EnergySourceContainer sources,
                          uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(this << ueDevs.GetN() << bwpIndex);
    NS_ASSERT_MSG(ueDevs.GetN() == sources.GetN(),
                  "One energy source per UE device is required (index-aligned)");

    energy::DeviceEnergyModelContainer models;
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = ueDevs.Get(i);
        Ptr<energy::EnergySource> source = sources.Get(i);
        NS_ASSERT_MSG(source->GetNode() == dev->GetNode(),
                      "Energy source " << i << " is not on the same node as UE device " << i
                                       << "; the two containers must be index-aligned");

        Ptr<NrUeNetDevice> nrDev = DynamicCast<NrUeNetDevice>(dev);
        NS_ABORT_MSG_IF(!nrDev, "NrEnergyHelper::InstallUe needs NrUeNetDevice objects");
        NS_ASSERT_MSG(bwpIndex < nrDev->GetCcMapSize(),
                      "UE device " << i << " has no bandwidth part " << bwpIndex);

        Ptr<NrUeEnergyModel> model = m_ueModelFactory.Create<NrUeEnergyModel>();
        AttachToSource(model, source);

        Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(dev, bwpIndex);
        Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
        listener->SetEnergyModel(model);
        listener->SetPhy(uePhy);

        if (m_enableDrx)
        {
            // The DRX model owns the sleep transitions; the listener restarts
            // its inactivity timer on each transport block.
            Ptr<NrUeDrxModel> drx = m_drxFactory.Create<NrUeDrxModel>();
            drx->SetEnergyModel(model);
            listener->SetDrxModel(drx);
            drx->Start(Simulator::Now());
            dev->GetNode()->AggregateObject(drx);
        }

        // On the PHY, not the node: one listener per PHY can never collide with
        // another of the same TypeId, which a node would not allow.
        uePhy->AggregateObject(listener);

        models.Add(model);
    }
    return models;
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::Install(NetDeviceContainer gnbDevs,
                        energy::EnergySourceContainer gnbSources,
                        NetDeviceContainer ueDevs,
                        energy::EnergySourceContainer ueSources)
{
    NS_LOG_FUNCTION(this);
    energy::DeviceEnergyModelContainer models = InstallGnb(gnbDevs, gnbSources);
    models.Add(InstallUe(ueDevs, ueSources));
    return models;
}

} // namespace ns3
