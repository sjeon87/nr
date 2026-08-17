// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_RADIO_SETUP_HELPER_H
#define NR_RADIO_SETUP_HELPER_H

#include "ns3/cc-bwp-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-helper.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

namespace ns3
{

/**
 * @brief Antenna profile used by NrRadioSetupHelper.
 */
struct NrAntennaProfile
{
  uint32_t gnbRows{4};     //!< Number of gNB antenna rows
  uint32_t gnbCols{8};     //!< Number of gNB antenna columns
  uint32_t ueRows{2};      //!< Number of UE antenna rows
  uint32_t ueCols{4};      //!< Number of UE antenna columns
  bool dualPolarized{false}; //!< Enable dual polarization for UE and gNB arrays
};

/**
 * @brief Grouped return value for installed radio devices.
 */
struct NrInstalledDevices
{
  NetDeviceContainer gnb; //!< Installed gNB devices
  NetDeviceContainer ue;  //!< Installed UE devices
};

/**
 * @ingroup helper
 * @brief Helper that centralizes repetitive NR radio setup steps.
 *
 * Rationale:
 * many examples repeat the same radio pattern:
 * - Set antenna attributes for gNB and UE
 * - Install gNB/UE devices
 * - Attach UEs to closest (or selected) gNB
 *
 * This helper keeps those repetitive operations in one place while preserving
 * scenario flexibility (e.g., BWP creation and channel setup remain in examples).
 */
class NrRadioSetupHelper : public Object
{
public:
  /**
   * @brief Get TypeId.
   * @return TypeId of NrRadioSetupHelper.
   */
  static TypeId GetTypeId();

  /**
   * @brief Apply a common antenna profile to NrHelper.
   * @param nrHelper NR helper instance.
   * @param profile Antenna profile to apply.
   */
  void ApplyAntennaProfile(Ptr<NrHelper> nrHelper, const NrAntennaProfile& profile);

  /**
   * @brief Install gNB and UE devices for a scenario.
   * @param nrHelper NR helper instance.
   * @param gnbNodes gNB nodes.
   * @param ueNodes UE nodes.
   * @param bwps BWP vector used by installation methods.
   * @return Installed gNB and UE device containers.
   */
  NrInstalledDevices InstallDevices(Ptr<NrHelper> nrHelper,
                                    const NodeContainer& gnbNodes,
                                    const NodeContainer& ueNodes,
                                    const BandwidthPartInfoPtrVector& bwps);

  /**
   * @brief Attach UEs to closest gNB.
   * @param nrHelper NR helper instance.
   * @param ueDevices UE devices.
   * @param gnbDevices gNB devices.
   */
  void AttachToClosest(Ptr<NrHelper> nrHelper,
                       const NetDeviceContainer& ueDevices,
                       const NetDeviceContainer& gnbDevices);
};

} // namespace ns3

#endif // NR_RADIO_SETUP_HELPER_H