// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_TRAFFIC_HELPER_H
#define NR_TRAFFIC_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-qos-flow.h"

namespace ns3
{

enum class NrTrafficDirection
{
  DOWNLINK,
  UPLINK
};

struct NrUdpFlowSpec
{
  NrTrafficDirection direction{NrTrafficDirection::DOWNLINK};
  uint16_t port{1234};
  uint32_t packetSize{100};
  Time interval{MilliSeconds(1)};
  Time start{Seconds(0.4)};
  Time stop{Seconds(1.0)};
  bool dedicatedQos{false};
  NrQosFlow::FiveQi fiveQi{NrQosFlow::NGBR_LOW_LAT_EMBB};
};

class NrTrafficHelper : public Object
{
public:
  static TypeId GetTypeId();

  ApplicationContainer InstallUdpFlow(const NrUdpFlowSpec& spec,
                                      Ptr<Node> remoteHost,
                                      const NodeContainer& ueNodes,
                                      const Ipv4InterfaceContainer& ueIfaces,
                                      Ptr<NrHelper> nrHelper,
                                      const NetDeviceContainer& ueDevices);
};

} // namespace ns3

#endif // NR_TRAFFIC_HELPER_H