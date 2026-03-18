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
#include "ns3/time.h"

namespace ns3
{

class Node;
class NrEpcHelper;

/**
 * @ingroup helper
 * @brief Helper that centralizes EPC + Internet boilerplate for NR examples.
 */
class NrInternetHelper : public Object
{
  public:
  static TypeId GetTypeId();

  NrInternetHelper();
  ~NrInternetHelper() override;

  void SetEpcHelper(Ptr<NrEpcHelper> epcHelper);

  void SetBackhaulAttributes(const DataRate& dataRate, uint16_t mtu, Time delay);

  void SetupRemoteHostIpv4();

  Ipv4InterfaceContainer AssignUeIpv4(const NetDeviceContainer& ueDevices) const;

  void SetupUeIpv4DefaultRoutes(const NodeContainer& ueNodes) const;

  Ptr<Node> GetRemoteHost() const;

  Ipv4Address GetRemoteHostAddress() const;

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