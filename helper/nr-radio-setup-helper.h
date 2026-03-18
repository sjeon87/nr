// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_RADIO_SETUP_HELPER_H
#define NR_RADIO_SETUP_HELPER_H

#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nr-helper.h"
#include "ns3/cc-bwp-helper.h"

namespace ns3
{

struct NrAntennaProfile
{
  uint32_t gnbRows{4};
  uint32_t gnbCols{8};
  uint32_t ueRows{2};
  uint32_t ueCols{4};
  bool dualPolarized{false};
};

struct NrInstalledDevices
{
  NetDeviceContainer gnb;
  NetDeviceContainer ue;
};

class NrRadioSetupHelper : public Object
{
public:
  static TypeId GetTypeId();

  void ApplyAntennaProfile(Ptr<NrHelper> nrHelper, const NrAntennaProfile& profile);

  NrInstalledDevices InstallDevices(Ptr<NrHelper> nrHelper,
                                    const NodeContainer& gnbNodes,
                                    const NodeContainer& ueNodes,
                                    const BandwidthPartInfoPtrVector& bwps);

  void AttachToClosest(Ptr<NrHelper> nrHelper,
                       const NetDeviceContainer& ueDevices,
                       const NetDeviceContainer& gnbDevices);
};

} // namespace ns3

#endif // NR_RADIO_SETUP_HELPER_H