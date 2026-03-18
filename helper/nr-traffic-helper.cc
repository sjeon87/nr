// SPDX-License-Identifier: GPL-2.0-only

#include "nr-traffic-helper.h"

#include "ns3/applications-module.h"
#include "ns3/address-value.h"
#include "ns3/log.h"
#include "ns3/nr-qos-rule.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrTrafficHelper");
NS_OBJECT_ENSURE_REGISTERED(NrTrafficHelper);

TypeId
NrTrafficHelper::GetTypeId()
{
  static TypeId tid = TypeId("ns3::NrTrafficHelper")
                        .SetParent<Object>()
                        .SetGroupName("Nr")
                        .AddConstructor<NrTrafficHelper>();
  return tid;
}

ApplicationContainer
NrTrafficHelper::InstallUdpFlow(const NrUdpFlowSpec& spec,
                                Ptr<Node> remoteHost,
                                const NodeContainer& ueNodes,
                                const Ipv4InterfaceContainer& ueIfaces,
                                Ptr<NrHelper> nrHelper,
                                const NetDeviceContainer& ueDevices)
{
  NS_ABORT_MSG_IF(remoteHost == nullptr, "NrTrafficHelper: remoteHost is null");
  NS_ABORT_MSG_IF(nrHelper == nullptr, "NrTrafficHelper: nrHelper is null");
  NS_ABORT_MSG_IF(ueNodes.GetN() != ueIfaces.GetN(), "UE nodes/interfaces size mismatch");

  UdpServerHelper sink(spec.port);
  ApplicationContainer serverApps = sink.Install(ueNodes);

  UdpClientHelper client;
  client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  client.SetAttribute("PacketSize", UintegerValue(spec.packetSize));
  client.SetAttribute("Interval", TimeValue(spec.interval));

  ApplicationContainer clientApps;
  for (uint32_t i = 0; i < ueIfaces.GetN(); ++i)
  {
    client.SetAttribute("RemoteAddress", AddressValue(ueIfaces.GetAddress(i)));
    client.SetAttribute("RemotePort", UintegerValue(spec.port));
    clientApps.Add(client.Install(remoteHost));

    if (spec.dedicatedQos)
    {
      NrQosFlow flow(spec.fiveQi);
      Ptr<NrQosRule> rule = Create<NrQosRule>();
      NrQosRule::PacketFilter pf;
      pf.localPortStart = spec.port;
      pf.localPortEnd = spec.port;
      pf.remotePortStart = spec.port;
      pf.remotePortEnd = spec.port;
      pf.direction = (spec.direction == NrTrafficDirection::UPLINK)
                       ? NrQosRule::UPLINK
                       : NrQosRule::DOWNLINK;
      rule->AddPacketFilter(pf);
      nrHelper->ActivateDedicatedQosFlow(ueDevices.Get(i), flow, rule);
    }
  }

  serverApps.Start(spec.start);
  clientApps.Start(spec.start);
  serverApps.Stop(spec.stop);
  clientApps.Stop(spec.stop);

  ApplicationContainer all;
  all.Add(serverApps);
  all.Add(clientApps);
  return all;
}