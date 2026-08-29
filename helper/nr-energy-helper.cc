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
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-gnb-phy-energy-listener.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-ue-drx-model.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nr-ue-phy-energy-listener.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrEnergyHelper");

namespace
{
// The energy listeners drive the first BWP of each device, matching the
// GetGnbPhy/GetUePhy(dev, 0) convention used throughout the NR examples.
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

        Ptr<NrGnbEnergyModel> model = m_gnbModelFactory.Create<NrGnbEnergyModel>();
        AttachToSource(model, source);

        // Wire the PHY listener: SetEnergyModel() before SetPhy() so the
        // listener can push the PHY's symbol duration into the model.
        Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
        listener->SetEnergyModel(model);
        listener->SetPhy(NrHelper::GetGnbPhy(dev, ENERGY_BWP_INDEX));

        // Keep the listener alive for the whole simulation: this helper is
        // usually stack-allocated and nothing else holds the listener.
        dev->GetNode()->AggregateObject(listener);

        models.Add(model);
    }
    return models;
}

energy::DeviceEnergyModelContainer
NrEnergyHelper::InstallUe(NetDeviceContainer ueDevs, energy::EnergySourceContainer sources)
{
    NS_LOG_FUNCTION(this << ueDevs.GetN());
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

        Ptr<NrUeEnergyModel> model = m_ueModelFactory.Create<NrUeEnergyModel>();
        AttachToSource(model, source);

        Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
        listener->SetEnergyModel(model);
        listener->SetPhy(NrHelper::GetUePhy(dev, ENERGY_BWP_INDEX));

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

        dev->GetNode()->AggregateObject(listener);

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
