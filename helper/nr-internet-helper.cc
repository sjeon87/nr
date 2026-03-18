// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-internet-helper.h"

#include "nr-epc-helper.h"

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/point-to-point-helper.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrInternetHelper");

NS_OBJECT_ENSURE_REGISTERED(NrInternetHelper);

TypeId
NrInternetHelper::GetTypeId()
{
  static TypeId tid =
    TypeId("ns3::NrInternetHelper")
      .SetParent<Object>()
      .SetGroupName("Nr")
      .AddConstructor<NrInternetHelper>();
  return tid;
}

NrInternetHelper::NrInternetHelper()
  : m_backhaulRate("100Gb/s"),
    m_backhaulMtu(2500),
    m_backhaulDelay(Seconds(0.0))
{
}

NrInternetHelper::~NrInternetHelper() = default;

void
NrInternetHelper::SetEpcHelper(Ptr<NrEpcHelper> epcHelper)
{
  m_epcHelper = epcHelper;
}

void
NrInternetHelper::SetBackhaulAttributes(const DataRate& dataRate, uint16_t mtu, Time delay)
{
  m_backhaulRate = dataRate;
  m_backhaulMtu = mtu;
  m_backhaulDelay = delay;
}

void
NrInternetHelper::SetupRemoteHostIpv4()
{
  NS_ABORT_MSG_IF(m_epcHelper == nullptr, "NrInternetHelper: EPC helper not set");

  Ptr<Node> pgw = m_epcHelper->GetPgwNode();

  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  m_remoteHost = remoteHostContainer.Get(0);

  InternetStackHelper internet;
  internet.Install(remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(m_backhaulRate));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(m_backhaulMtu));
  p2ph.SetChannelAttribute("Delay", TimeValue(m_backhaulDelay));

  NetDeviceContainer internetDevices = p2ph.Install(pgw, m_remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  m_remoteHostAddr = internetIpIfaces.GetAddress(1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
    ipv4RoutingHelper.GetStaticRouting(m_remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
}

Ipv4InterfaceContainer
NrInternetHelper::AssignUeIpv4(const NetDeviceContainer& ueDevices) const
{
  NS_ABORT_MSG_IF(m_epcHelper == nullptr, "NrInternetHelper: EPC helper not set");
  return m_epcHelper->AssignUeIpv4Address(ueDevices);
}

void
NrInternetHelper::SetupUeIpv4DefaultRoutes(const NodeContainer& ueNodes) const
{
  NS_ABORT_MSG_IF(m_epcHelper == nullptr, "NrInternetHelper: EPC helper not set");

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  const Ipv4Address gatewayAddress = m_epcHelper->GetUeDefaultGatewayAddress();
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    Ptr<Ipv4StaticRouting> ueStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(gatewayAddress, 1);
  }
}

Ptr<Node>
NrInternetHelper::GetRemoteHost() const
{
  return m_remoteHost;
}

Ipv4Address
NrInternetHelper::GetRemoteHostAddress() const
{
  return m_remoteHostAddr;
}

} // namespace ns3