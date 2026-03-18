// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_INTERNET_HELPER_H
#define NR_INTERNET_HELPER_H

#include "ns3/data-rate.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"

#include <string>

namespace ns3
{
/**
 * @brief Aggregated endpoints returned by SetupFullInternet().
 */
struct NrInternetEndpoints
{
  Ptr<Node> remoteHost;              //!< Remote host node connected to PGW
  Ipv4Address remoteHostAddress;     //!< IPv4 address assigned to remote host PGW link
  Ipv4InterfaceContainer ueIfaces;   //!< IPv4 interfaces assigned to UE devices
};
class Node;
class NrEpcHelper;

/**
 * @ingroup helper
 * @brief Helper that centralizes EPC + IPv4 setup boilerplate for NR examples.
 *
 * This helper is intended to reduce repeated scenario code for:
 * - PGW to remote-host point-to-point connectivity
 * - UE IPv4 address assignment through EPC
 * - UE default route configuration towards EPC gateway
 *
 * Users can call either granular methods (SetupRemoteHostIpv4(), AssignUeIpv4(),
 * SetupUeIpv4DefaultRoutes()) or a one-shot method (SetupFullInternet()) that
 * applies the full sequence in the correct order.
 */
class NrInternetHelper : public Object
{
  public:
  /**
   * @brief Get type ID.
   * @return TypeId of this helper.
   */
  static TypeId GetTypeId();

  /**
   * @brief Construct the helper with default backhaul attributes.
   */
  NrInternetHelper();

  /**
   * @brief Destroy the helper.
   */
  ~NrInternetHelper() override;

  /**
   * @brief Set the EPC helper used by this class.
   * @param epcHelper EPC helper instance.
   */
  void SetEpcHelper(Ptr<NrEpcHelper> epcHelper);

  /**
   * @brief Set PGW<->remote-host link attributes.
   * @param dataRate Link data rate.
   * @param mtu Link MTU.
   * @param delay Link propagation delay.
   */
  void SetBackhaulAttributes(const DataRate& dataRate, uint16_t mtu, Time delay);

  /**
   * @brief Create remote host and connect it to PGW with IPv4 configuration.
   */
  void SetupRemoteHostIpv4();

  /**
   * @brief Assign IPv4 addresses to UE devices through EPC.
   * @param ueDevices UE net devices.
   * @return UE IPv4 interface container.
   */
  Ipv4InterfaceContainer AssignUeIpv4(const NetDeviceContainer& ueDevices) const;

  /**
   * @brief Configure UE default routes towards EPC gateway.
   * @param ueNodes UE node container.
   */
  void SetupUeIpv4DefaultRoutes(const NodeContainer& ueNodes) const;

  /**
   * @brief Get the created remote host node.
   * @return Remote host node pointer.
   */
  Ptr<Node> GetRemoteHost() const;

  /**
   * @brief Get IPv4 address of the created remote host.
   * @return Remote host IPv4 address.
   */
  Ipv4Address GetRemoteHostAddress() const;

  /**
   * @brief One-shot internet setup for NR + EPC examples.
   *
   * The method performs, in order:
   * - remote-host creation and PGW connectivity
   * - UE internet stack installation
   * - UE IPv4 assignment through EPC
   * - remote-host route to UE subnet
   * - UE default-route configuration to EPC gateway
   *
   * @param ueNodes UE nodes to configure.
   * @param ueDevices UE net devices matching ueNodes.
   * @param pgwSubnet Base subnet for PGW<->remote-host link.
   * @param pgwMask Subnet mask for PGW<->remote-host link.
   * @param ueSubnet UE network/subnet reachable through PGW.
   * @param ueMask Subnet mask for UE network.
   * @return A struct containing remote host and UE interface handles.
   */
  NrInternetEndpoints SetupFullInternet(const NodeContainer& ueNodes,
                                        const NetDeviceContainer& ueDevices,
                                        const std::string& pgwSubnet = "1.0.0.0",
                                        const std::string& pgwMask = "255.0.0.0",
                                        const std::string& ueSubnet = "7.0.0.0",
                                        const std::string& ueMask = "255.0.0.0");
  private:
  Ptr<NrEpcHelper> m_epcHelper;
  Ptr<Node> m_remoteHost;
  Ipv4Address m_remoteHostAddr;

  DataRate m_backhaulRate;
  uint16_t m_backhaulMtu;
  Time m_backhaulDelay;
};

} // namespace ns3

#endif // NR_INTERNET_HELPER_H