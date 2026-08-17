// SPDX-License-Identifier: GPL-2.0-only

#include "nr-radio-setup-helper.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRadioSetupHelper");
NS_OBJECT_ENSURE_REGISTERED(NrRadioSetupHelper);

TypeId
NrRadioSetupHelper::GetTypeId()
{
  static TypeId tid = TypeId("ns3::NrRadioSetupHelper")
                        .SetParent<Object>()
                        .SetGroupName("Nr")
                        .AddConstructor<NrRadioSetupHelper>();
  return tid;
}

void
NrRadioSetupHelper::ApplyAntennaProfile(Ptr<NrHelper> nrHelper, const NrAntennaProfile& profile)
{
  NS_ABORT_MSG_IF(nrHelper == nullptr, "NrRadioSetupHelper: nrHelper is null");

  // Apply a compact antenna profile used across many NR examples.
  nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(profile.gnbRows));
  nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(profile.gnbCols));
  nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(profile.ueRows));
  nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(profile.ueCols));

  if (profile.dualPolarized)
  {
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(true));
  }
}

NrInstalledDevices
NrRadioSetupHelper::InstallDevices(Ptr<NrHelper> nrHelper,
                                   const NodeContainer& gnbNodes,
                                   const NodeContainer& ueNodes,
                                   const BandwidthPartInfoPtrVector& bwps)
{
  NS_ABORT_MSG_IF(nrHelper == nullptr, "NrRadioSetupHelper: nrHelper is null");
  // Keep gNB/UE install sequence in one call site to reduce boilerplate.
  NrInstalledDevices out;
  out.gnb = nrHelper->InstallGnbDevice(gnbNodes, bwps);
  out.ue = nrHelper->InstallUeDevice(ueNodes, bwps);
  return out;
}

void
NrRadioSetupHelper::AttachToClosest(Ptr<NrHelper> nrHelper,
                                    const NetDeviceContainer& ueDevices,
                                    const NetDeviceContainer& gnbDevices)
{
  NS_ABORT_MSG_IF(nrHelper == nullptr, "NrRadioSetupHelper: nrHelper is null");
  nrHelper->AttachToClosestGnb(ueDevices, gnbDevices);
}

} // namespace ns3