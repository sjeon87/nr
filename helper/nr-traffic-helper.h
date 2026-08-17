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

/**
 * @brief Direction of configured traffic flow.
 */
enum class NrTrafficDirection
{
  DOWNLINK, //!< Traffic from remote host to UE
  UPLINK    //!< Traffic from UE to remote host
};

/**
 * @brief UDP flow specification used by NrTrafficHelper.
 */
struct NrUdpFlowSpec
{
  NrTrafficDirection direction{NrTrafficDirection::DOWNLINK}; //!< Traffic direction
  uint16_t port{1234};                                        //!< UDP destination/source port
  uint32_t packetSize{100};                                  //!< UDP payload size in bytes
  Time interval{MilliSeconds(1)};                            //!< Inter-packet interval
  Time start{Seconds(0.4)};                                  //!< Application start time
  Time stop{Seconds(1.0)};                                   //!< Application stop time
  bool dedicatedQos{false};                                  //!< Enable dedicated QoS activation
  NrQosFlow::FiveQi fiveQi{NrQosFlow::NGBR_LOW_LAT_EMBB};    //!< 5QI used when dedicatedQos=true
};

/**
 * @ingroup helper
 * @brief Helper that centralizes repetitive NR UDP traffic and QoS setup.
 *
 * Rationale:
 * many NR examples repeat the same traffic blocks:
 * - UdpServerHelper/UdpClientHelper setup
 * - per-UE app installation loops
 * - optional NrQosFlow + NrQosRule + ActivateDedicatedQosFlow
 *
 * This helper provides a compact flow spec to reduce duplicated code while
 * preserving explicit control over QoS activation and timing.
 */
class NrTrafficHelper : public Object
{
public:
  /**
   * @brief Get TypeId.
   * @return TypeId of NrTrafficHelper.
   */
  static TypeId GetTypeId();

  /**
   * @brief Install one UDP flow pattern across all UE endpoints.
   * @param spec Flow specification.
   * @param remoteHost Remote host used as traffic endpoint.
   * @param ueNodes UE nodes used for sink installation.
   * @param ueIfaces UE IPv4 interfaces.
   * @param nrHelper NR helper used for optional dedicated QoS activation.
   * @param ueDevices UE net devices.
   * @return Container with installed server and client applications.
   */
  ApplicationContainer InstallUdpFlow(const NrUdpFlowSpec& spec,
                                      Ptr<Node> remoteHost,
                                      const NodeContainer& ueNodes,
                                      const Ipv4InterfaceContainer& ueIfaces,
                                      Ptr<NrHelper> nrHelper,
                                      const NetDeviceContainer& ueDevices);
};

} // namespace ns3

#endif // NR_TRAFFIC_HELPER_H