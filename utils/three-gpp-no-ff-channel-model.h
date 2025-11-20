/*
 * Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ns3/three-gpp-channel-model.h"

#ifndef NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H
#define NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H

namespace ns3
{

class ThreeGppNoFFChannelModel : public ThreeGppChannelModel
{
  public:
    /**
     * Constructor
     */
    ThreeGppNoFFChannelModel();

    /**
     * Destructor
     */
    ~ThreeGppNoFFChannelModel() override;

    /**
     * Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    virtual Ptr<ChannelMatrix> GetNewChannel(Ptr<const ThreeGppChannelParams> channelParams,
                                             Ptr<const ParamsTable> table3gpp,
                                             const Ptr<const MobilityModel> sMob,
                                             const Ptr<const MobilityModel> uMob,
                                             Ptr<const PhasedArrayModel> sAntenna,
                                             Ptr<const PhasedArrayModel> uAntenna) const override;
};

} // namespace ns3

#endif // NS3_THREE_GPP_NO_FF_CHANNEL_MODEL_H
