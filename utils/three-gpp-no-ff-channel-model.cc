/*
 * Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "three-gpp-no-ff-channel-model.h"

#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThreeGppNoFFChannelModel");

NS_OBJECT_ENSURE_REGISTERED(ThreeGppNoFFChannelModel);

ThreeGppNoFFChannelModel::ThreeGppNoFFChannelModel()
{
    NS_LOG_FUNCTION(this);
}

ThreeGppNoFFChannelModel::~ThreeGppNoFFChannelModel()
{
    NS_LOG_FUNCTION(this);
}

TypeId
ThreeGppNoFFChannelModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThreeGppNoFFChannelModel")
                            .SetParent<ThreeGppChannelModel>()
                            .SetGroupName("Spectrum")
                            .AddConstructor<ThreeGppNoFFChannelModel>();
    return tid;
}

Ptr<MatrixBasedChannelModel::ChannelMatrix>
ThreeGppNoFFChannelModel::GetNewChannel(Ptr<const ThreeGppChannelParams> channelParams,
                                        Ptr<const ParamsTable> table3gpp,
                                        const Ptr<const MobilityModel> sMob,
                                        const Ptr<const MobilityModel> uMob,
                                        Ptr<const PhasedArrayModel> sAntenna,
                                        Ptr<const PhasedArrayModel> uAntenna) const
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(m_frequency > 0.0, "Set the operating frequency first!");

    // create a channel matrix instance
    Ptr<ChannelMatrix> channelMatrix = Create<ChannelMatrix>();
    channelMatrix->m_generatedTime = Simulator::Now();
    // save in which order is generated this matrix
    channelMatrix->m_nodeIds =
        std::make_pair(sMob->GetObject<Node>()->GetId(), uMob->GetObject<Node>()->GetId());
    channelMatrix->m_antennaPair =
        std::make_pair(sAntenna->GetId(),
                       uAntenna->GetId()); // save antenna pair, with the exact order of s and u

    // Step 11: Generate channel coefficients for each cluster n and each receiver
    //  and transmitter element pair u,s.
    // where n is cluster index, u and s are receive and transmit antenna element.
    size_t uSize = uAntenna->GetNumElems();
    size_t sSize = sAntenna->GetNumElems();

    // NOTE: Since each of the strongest 2 clusters are divided into 3 sub-clusters,
    // the total cluster will generally be numReducedCLuster + 4.
    // However, it might be that m_cluster1st = m_cluster2nd. In this case the
    // total number of clusters will be numReducedCLuster + 2.
    uint16_t numOverallCluster = (channelParams->m_cluster1st != channelParams->m_cluster2nd)
                                     ? channelParams->m_reducedClusterNumber + 4
                                     : channelParams->m_reducedClusterNumber + 2;

    Complex3DVector hUsn(uSize, sSize, numOverallCluster);
    // channel coefficient hUsn (u, s, n);
    std::complex<double> one(1.0, 0.0);
    std::complex<double> zero(0.0, 0.0);

    for (size_t cIndex = 0; cIndex < hUsn.GetNumPages(); cIndex++)
    {
        for (size_t rowIdx = 0; rowIdx < hUsn.GetNumRows(); rowIdx++)
        {
            for (size_t colIdx = 0; colIdx < hUsn.GetNumCols(); colIdx++)
            {
                hUsn(rowIdx, colIdx, cIndex) = (cIndex == 0) ? one : zero;
                NS_LOG_DEBUG(" " << hUsn(rowIdx, colIdx, cIndex) << ",");
            }
        }
    }

    NS_LOG_INFO("size of coefficient matrix (rows, columns, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")");
    channelMatrix->m_channel = hUsn;
    return channelMatrix;
}

} // namespace ns3
