/*
 * Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "three-gpp-no-ff-channel-model.h"

#include "ns3/double.h"
#include "ns3/enum.h"
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
    static TypeId tid =
        TypeId("ns3::ThreeGppNoFFChannelModel")
            .SetParent<ThreeGppChannelModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<ThreeGppNoFFChannelModel>()
            .AddAttribute("NoFFModelType",
                          "Different calibration types to exclude fast fading",
                          EnumValue(ThreeGppNoFFChannelModel::ALL_ONES),
                          MakeEnumAccessor<CalibNoFfHsn>(&ThreeGppNoFFChannelModel::m_noFfModel),
                          MakeEnumChecker(ThreeGppNoFFChannelModel::ALL_ONES,
                                          "all_ones",
                                          ThreeGppNoFFChannelModel::CHATGPT,
                                          "chatgpt",
                                          ThreeGppNoFFChannelModel::SLAGEN,
                                          "slagen",
                                          ThreeGppNoFFChannelModel::GROK,
                                          "grok"));
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
    switch (m_noFfModel)
    {
    case CalibNoFfHsn::ALL_ONES:
        return GetAllOnesNoFF(channelParams, table3gpp, sMob, uMob, sAntenna, uAntenna);
        break;
    case CalibNoFfHsn::CHATGPT:
        return GetChatGptNoFF(channelParams, table3gpp, sMob, uMob, sAntenna, uAntenna);
        break;
    case CalibNoFfHsn::SLAGEN:
        return GetSLagenNoFF(channelParams, table3gpp, sMob, uMob, sAntenna, uAntenna);
        break;
    case CalibNoFfHsn::GROK:
        return GetGrokNoFF(channelParams, table3gpp, sMob, uMob, sAntenna, uAntenna);
        break;
    default:
        NS_ABORT_MSG("Not supported yet");
    }
}

Ptr<MatrixBasedChannelModel::ChannelMatrix>
ThreeGppNoFFChannelModel::GetAllOnesNoFF(Ptr<const ThreeGppChannelParams> channelParams,
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

    double channelNorm = 0.0;
    NS_LOG_DEBUG("Husn (sAntenna, uAntenna):" << sAntenna->GetId() << ", " << uAntenna->GetId());
    for (size_t cIndex = 0; cIndex < hUsn.GetNumPages(); cIndex++)
    {
        double clusterNorm = 0.0;
        for (size_t rowIdx = 0; rowIdx < hUsn.GetNumRows(); rowIdx++)
        {
            for (size_t colIdx = 0; colIdx < hUsn.GetNumCols(); colIdx++)
            {
                clusterNorm += std::pow(std::abs(hUsn(rowIdx, colIdx, cIndex)), 2);
                NS_LOG_DEBUG(" " << hUsn(rowIdx, colIdx, cIndex) << ",");
            }
        }
        channelNorm += clusterNorm;
    }

    NS_LOG_INFO("size of coefficient matrix (rows, columns, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")" << "Channel norm " << channelNorm);

    channelMatrix->m_channel = hUsn;
    return channelMatrix;
}

Ptr<MatrixBasedChannelModel::ChannelMatrix>
ThreeGppNoFFChannelModel::GetChatGptNoFF(Ptr<const ThreeGppChannelParams> channelParams,
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
    // where n is cluster index, u and s are receive and transmit antenna element.
    size_t uSize = uAntenna->GetNumElems();
    size_t sSize = sAntenna->GetNumElems();
    double x = sMob->GetPosition().x - uMob->GetPosition().x;
    double y = sMob->GetPosition().y - uMob->GetPosition().y;
    double distance2D = sqrt(x * x + y * y);
    // NOTE we assume hUT = min (height(a), height(b)) and
    // hBS = max (height (a), height (b))
    double hUt = std::min(sMob->GetPosition().z, uMob->GetPosition().z);
    double hBs = std::max(sMob->GetPosition().z, uMob->GetPosition().z);
    // compute the 3D distance using eq. 7.4-1
    double distance3D = std::sqrt(distance2D * distance2D + (hBs - hUt) * (hBs - hUt));

    Angles sAngle(uMob->GetPosition(), sMob->GetPosition());
    Angles uAngle(sMob->GetPosition(), uMob->GetPosition());

    // ----------------------------------------------------------------------
    // NEW: deterministic, no-fast-fading mode
    // ----------------------------------------------------------------------

    NS_LOG_INFO("ThreeGppChannelModel: DisableFastFading = true, "
                "building deterministic H with array pattern and geometry only.");

    // Single deterministic cluster/tap
    uint16_t numOverallCluster = 1;
    Complex3DVector hUsn(uSize, sSize, numOverallCluster);

    double lambda = 3.0e8 / m_frequency;

    const double sinUAngleIncl = std::sin(uAngle.GetInclination());
    const double cosUAngleIncl = std::cos(uAngle.GetInclination());
    const double sinUAngleAz = std::sin(uAngle.GetAzimuth());
    const double cosUAngleAz = std::cos(uAngle.GetAzimuth());

    const double sinSAngleIncl = std::sin(sAngle.GetInclination());
    const double cosSAngleIncl = std::cos(sAngle.GetInclination());
    const double sinSAngleAz = std::sin(sAngle.GetAzimuth());
    const double cosSAngleAz = std::cos(sAngle.GetAzimuth());

    // optional: deterministic propagation phase for the distance
    double propPhase = -2 * M_PI * distance3D / lambda;
    std::complex<double> distPhase(std::cos(propPhase), std::sin(propPhase));

    for (size_t uIndex = 0; uIndex < uSize; ++uIndex)
    {
        Vector uLoc = uAntenna->GetElementLocation(uIndex);
        double rxPhaseDiff = 2 * M_PI / lambda *
                             (sinUAngleIncl * cosUAngleAz * uLoc.x +
                              sinUAngleIncl * sinUAngleAz * uLoc.y + cosUAngleIncl * uLoc.z);

        auto [rxFieldPhi, rxFieldTheta] =
            uAntenna->GetElementFieldPattern(Angles(uAngle.GetAzimuth(), uAngle.GetInclination()),
                                             uAntenna->GetElemPol(uIndex));

        for (size_t sIndex = 0; sIndex < sSize; ++sIndex)
        {
            Vector sLoc = sAntenna->GetElementLocation(sIndex);
            double txPhaseDiff = 2 * M_PI / lambda *
                                 (sinSAngleIncl * cosSAngleAz * sLoc.x +
                                  sinSAngleIncl * sinSAngleAz * sLoc.y + cosSAngleIncl * sLoc.z);

            auto [txFieldPhi, txFieldTheta] = sAntenna->GetElementFieldPattern(
                Angles(sAngle.GetAzimuth(), sAngle.GetInclination()),
                sAntenna->GetElemPol(sIndex));

            // Same polarization combination as LoS block: theta-theta - phi-phi
            std::complex<double> pat = (rxFieldTheta * txFieldTheta - rxFieldPhi * txFieldPhi);

            std::complex<double> phaseTerm(std::cos(rxPhaseDiff + txPhaseDiff),
                                           std::sin(rxPhaseDiff + txPhaseDiff));

            // NO pathloss factor here: pathloss/shadowing handled by propagation loss model
            hUsn(uIndex, sIndex, 0) = pat * phaseTerm * distPhase;
        }
    }

    double channelNorm = 0.0;
    NS_LOG_DEBUG("Husn (sAntenna, uAntenna):" << sAntenna->GetId() << ", " << uAntenna->GetId());
    for (size_t cIndex = 0; cIndex < hUsn.GetNumPages(); cIndex++)
    {
        double clusterNorm = 0.0;
        for (size_t rowIdx = 0; rowIdx < hUsn.GetNumRows(); rowIdx++)
        {
            for (size_t colIdx = 0; colIdx < hUsn.GetNumCols(); colIdx++)
            {
                clusterNorm += std::pow(std::abs(hUsn(rowIdx, colIdx, cIndex)), 2);
                NS_LOG_DEBUG(" " << hUsn(rowIdx, colIdx, cIndex) << ",");
            }
        }
        channelNorm += clusterNorm;
    }

    NS_LOG_INFO("size of coefficient matrix (rows, columns, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")" << "Channel norm " << channelNorm);
    channelMatrix->m_channel = hUsn;
    return channelMatrix;
}

Ptr<MatrixBasedChannelModel::ChannelMatrix>
ThreeGppNoFFChannelModel::GetSLagenNoFF(Ptr<const ThreeGppChannelParams> channelParams,
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
    // where n is cluster index, u and s are receive and transmit antenna element.
    size_t uSize = uAntenna->GetNumElems();
    size_t sSize = sAntenna->GetNumElems();
    double x = sMob->GetPosition().x - uMob->GetPosition().x;
    double y = sMob->GetPosition().y - uMob->GetPosition().y;
    double distance2D = sqrt(x * x + y * y);
    // NOTE we assume hUT = min (height(a), height(b)) and
    // hBS = max (height (a), height (b))
    double hUt = std::min(sMob->GetPosition().z, uMob->GetPosition().z);
    double hBs = std::max(sMob->GetPosition().z, uMob->GetPosition().z);
    // compute the 3D distance using eq. 7.4-1
    double distance3D = std::sqrt(distance2D * distance2D + (hBs - hUt) * (hBs - hUt));

    Angles sAngle(uMob->GetPosition(), sMob->GetPosition());
    Angles uAngle(sMob->GetPosition(), uMob->GetPosition());

    // ----------------------------------------------------------------------
    // NEW: deterministic, no-fast-fading mode
    // ----------------------------------------------------------------------

    NS_LOG_INFO("ThreeGppChannelModel: DisableFastFading = true, "
                "building deterministic H with array pattern and geometry only (uniform across "
                "MIMO elements).");

    // Single deterministic cluster/tap
    uint16_t numOverallCluster = 1;
    Complex3DVector hUsn(uSize, sSize, numOverallCluster);

    double lambda = 3.0e8 / m_frequency;

    const double sinUAngleIncl = std::sin(uAngle.GetInclination());
    const double cosUAngleIncl = std::cos(uAngle.GetInclination());
    const double sinUAngleAz = std::sin(uAngle.GetAzimuth());
    const double cosUAngleAz = std::cos(uAngle.GetAzimuth());

    const double sinSAngleIncl = std::sin(sAngle.GetInclination());
    const double cosSAngleIncl = std::cos(sAngle.GetInclination());
    const double sinSAngleAz = std::sin(sAngle.GetAzimuth());
    const double cosSAngleAz = std::cos(sAngle.GetAzimuth());

    // optional: deterministic propagation phase for the distance
    double propPhase = -2 * M_PI * distance3D / lambda;
    std::complex<double> distPhase(std::cos(propPhase), std::sin(propPhase));

    // Compute centroid of RX (u) antenna element positions
    Vector uCentroid(0.0, 0.0, 0.0);
    for (size_t i = 0; i < uSize; ++i)
    {
        Vector loc = uAntenna->GetElementLocation(i);
        uCentroid.x += loc.x;
        uCentroid.y += loc.y;
        uCentroid.z += loc.z;
    }
    if (uSize > 0)
    {
        uCentroid.x /= static_cast<double>(uSize);
        uCentroid.y /= static_cast<double>(uSize);
        uCentroid.z /= static_cast<double>(uSize);
    }

    // Compute centroid of TX (s) antenna element positions
    Vector sCentroid(0.0, 0.0, 0.0);
    for (size_t j = 0; j < sSize; ++j)
    {
        Vector loc = sAntenna->GetElementLocation(j);
        sCentroid.x += loc.x;
        sCentroid.y += loc.y;
        sCentroid.z += loc.z;
    }
    if (sSize > 0)
    {
        sCentroid.x /= static_cast<double>(sSize);
        sCentroid.y /= static_cast<double>(sSize);
        sCentroid.z /= static_cast<double>(sSize);
    }

    // Phase difference for centroid positions (represents main direction geometry)
    double rxCentPhaseDiff =
        2 * M_PI / lambda *
        (sinUAngleIncl * cosUAngleAz * uCentroid.x + sinUAngleIncl * sinUAngleAz * uCentroid.y +
         cosUAngleIncl * uCentroid.z);

    double txCentPhaseDiff =
        2 * M_PI / lambda *
        (sinSAngleIncl * cosSAngleAz * sCentroid.x + sinSAngleIncl * sinSAngleAz * sCentroid.y +
         cosSAngleIncl * sCentroid.z);

    // Use element 0 polarization as representative (centroid has no element index)
    uint16_t uPolIdx = (uSize > 0) ? uAntenna->GetElemPol(0) : 0;
    uint16_t sPolIdx = (sSize > 0) ? sAntenna->GetElemPol(0) : 0;

    // Field pattern at the centroid direction (main lobe)
    auto [rxFieldPhiCent, rxFieldThetaCent] =
        uAntenna->GetElementFieldPattern(Angles(uAngle.GetAzimuth(), uAngle.GetInclination()),
                                         uPolIdx);

    auto [txFieldPhiCent, txFieldThetaCent] =
        sAntenna->GetElementFieldPattern(Angles(sAngle.GetAzimuth(), sAngle.GetInclination()),
                                         sPolIdx);

    // Same polarization combination as LoS block: theta-theta - phi-phi
    std::complex<double> patCent =
        (rxFieldThetaCent * txFieldThetaCent - rxFieldPhiCent * txFieldPhiCent);

    std::complex<double> phaseCent(std::cos(rxCentPhaseDiff + txCentPhaseDiff),
                                   std::sin(rxCentPhaseDiff + txCentPhaseDiff));

    // The single value for all MIMO elements (no per-element fast fading)
    std::complex<double> uniformCoeff = patCent * phaseCent * distPhase;

    // Fill the entire MIMO matrix with the same coefficient
    for (size_t uIndex = 0; uIndex < uSize; ++uIndex)
    {
        for (size_t sIndex = 0; sIndex < sSize; ++sIndex)
        {
            hUsn(uIndex, sIndex, 0) = uniformCoeff;
        }
    }

    NS_LOG_INFO("Built uniform deterministic coefficient matrix (rows, cols, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")");
    NS_LOG_INFO("Uniform coefficient (real, imag) = (" << uniformCoeff.real() << ", "
                                                       << uniformCoeff.imag() << ")");

    double channelNorm = 0.0;
    NS_LOG_DEBUG("Husn (sAntenna, uAntenna):" << sAntenna->GetId() << ", " << uAntenna->GetId());
    for (size_t cIndex = 0; cIndex < hUsn.GetNumPages(); cIndex++)
    {
        double clusterNorm = 0.0;
        for (size_t rowIdx = 0; rowIdx < hUsn.GetNumRows(); rowIdx++)
        {
            for (size_t colIdx = 0; colIdx < hUsn.GetNumCols(); colIdx++)
            {
                clusterNorm += std::pow(std::abs(hUsn(rowIdx, colIdx, cIndex)), 2);
                NS_LOG_DEBUG(" " << hUsn(rowIdx, colIdx, cIndex) << ",");
            }
        }
        channelNorm += clusterNorm;
    }

    NS_LOG_INFO("size of coefficient matrix (rows, columns, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")" << "Channel norm " << channelNorm);
    channelMatrix->m_channel = hUsn;
    return channelMatrix;
}

Ptr<MatrixBasedChannelModel::ChannelMatrix>
ThreeGppNoFFChannelModel::GetGrokNoFF(Ptr<const ThreeGppChannelParams> channelParams,
                                      Ptr<const ParamsTable> table3gpp,
                                      const Ptr<const MobilityModel> sMob,
                                      const Ptr<const MobilityModel> uMob,
                                      Ptr<const PhasedArrayModel> sAntenna,
                                      Ptr<const PhasedArrayModel> uAntenna) const
{
    NS_LOG_FUNCTION(this);
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(m_frequency > 0.0, "Set the operating frequency first!");

    // create a channel matrix instance
    Ptr<ChannelMatrix> channelMatrix = Create<ChannelMatrix>();
    channelMatrix->m_generatedTime = Simulator::Now();
    // save in which order is generated this matrix
    channelMatrix->m_nodeIds =
        std::make_pair(sMob->GetObject<Node>()->GetId(), uMob->GetObject<Node>()->GetId());
    // where n is cluster index, u and s are receive and transmit antenna element.
    size_t uSize = uAntenna->GetNumElems();
    size_t sSize = sAntenna->GetNumElems();
    double x = sMob->GetPosition().x - uMob->GetPosition().x;
    double y = sMob->GetPosition().y - uMob->GetPosition().y;
    double distance2D = sqrt(x * x + y * y);
    // NOTE we assume hUT = min (height(a), height(b)) and
    // hBS = max (height (a), height (b))
    double hUt = std::min(sMob->GetPosition().z, uMob->GetPosition().z);
    double hBs = std::max(sMob->GetPosition().z, uMob->GetPosition().z);
    // compute the 3D distance using eq. 7.4-1
    double distance3D = std::sqrt(distance2D * distance2D + (hBs - hUt) * (hBs - hUt));

    Angles sAngle(uMob->GetPosition(), sMob->GetPosition());
    Angles uAngle(sMob->GetPosition(), uMob->GetPosition());

    // Single deterministic cluster/tap
    uint16_t numOverallCluster = 1;
    Complex3DVector hUsn(uSize, sSize, numOverallCluster);

    double lambda = 3.0e8 / m_frequency;

    const double sinUAngleIncl = std::sin(uAngle.GetInclination());
    const double cosUAngleIncl = std::cos(uAngle.GetInclination());
    const double sinUAngleAz = std::sin(uAngle.GetAzimuth());
    const double cosUAngleAz = std::cos(uAngle.GetAzimuth());

    const double sinSAngleIncl = std::sin(sAngle.GetInclination());
    const double cosSAngleIncl = std::cos(sAngle.GetInclination());
    const double sinSAngleAz = std::sin(sAngle.GetAzimuth());
    const double cosSAngleAz = std::cos(sAngle.GetAzimuth());

    double propPhase = -2 * M_PI * distance3D / lambda;
    std::complex<double> phaseDiffDueToDistance(std::cos(propPhase), std::sin(propPhase));

    // Note: Path loss and shadowing are applied separately in the propagation loss model.
    // Here, normalize the matrix such that effective |h|^2 ~1 (deterministic spatial phases only).

    for (size_t uIndex = 0; uIndex < uSize; uIndex++)
    {
        Vector uLoc = uAntenna->GetElementLocation(uIndex);
        double rxPhaseDiff = 2 * M_PI *
                             (sinUAngleIncl * cosUAngleAz * uLoc.x +
                              sinUAngleIncl * sinUAngleAz * uLoc.y + cosUAngleIncl * uLoc.z);

        auto [rxFieldPatternPhi, rxFieldPatternTheta] =
            uAntenna->GetElementFieldPattern(uAngle,
                                             // Use direct arrival angle
                                             uAntenna->GetElemPol(uIndex));

        for (size_t sIndex = 0; sIndex < sSize; sIndex++)
        {
            Vector sLoc = sAntenna->GetElementLocation(sIndex);
            double txPhaseDiff = 2 * M_PI *
                                 (sinSAngleIncl * cosSAngleAz * sLoc.x +
                                  sinSAngleIncl * sinSAngleAz * sLoc.y + cosSAngleIncl * sLoc.z);

            auto [txFieldPatternPhi, txFieldPatternTheta] =
                sAntenna->GetElementFieldPattern(sAngle,
                                                 // Use direct departure angle
                                                 sAntenna->GetElemPol(sIndex));

            // Deterministic direct ray: LOS-like polarization (7.5-29), fixed phase 0, no cross-pol
            // ratios or random phases Co-pol theta-theta minus phi-phi for direct path; normalize
            // magnitude to 1 (spatial structure preserved)
            std::complex<double> detRay = (rxFieldPatternTheta * txFieldPatternTheta -
                                           rxFieldPatternPhi * txFieldPatternPhi) *
                                          phaseDiffDueToDistance *
                                          std::complex<double>(cos(rxPhaseDiff), sin(rxPhaseDiff)) *
                                          std::complex<double>(cos(txPhaseDiff), sin(txPhaseDiff));

            // Normalize: divide by magnitude of the pattern product to ensure |detRay| = 1 (only
            // phases from geometry) This keeps the matrix normalized, removing amplitude fading
            // while preserving MIMO phases
            double patternMag = std::abs(rxFieldPatternTheta * txFieldPatternTheta -
                                         rxFieldPatternPhi * txFieldPatternPhi);
            if (patternMag > 0.0)
            {
                detRay /= patternMag;
            }
            // Else, detRay remains 0 (poor directivity)

            hUsn(uIndex, sIndex, 0) = detRay; // Single cluster, normalized
        }
    }

    double channelNorm = 0.0;
    NS_LOG_DEBUG("Husn (sAntenna, uAntenna):" << sAntenna->GetId() << ", " << uAntenna->GetId());
    for (size_t cIndex = 0; cIndex < hUsn.GetNumPages(); cIndex++)
    {
        double clusterNorm = 0.0;
        for (size_t rowIdx = 0; rowIdx < hUsn.GetNumRows(); rowIdx++)
        {
            for (size_t colIdx = 0; colIdx < hUsn.GetNumCols(); colIdx++)
            {
                clusterNorm += std::pow(std::abs(hUsn(rowIdx, colIdx, cIndex)), 2);
                NS_LOG_DEBUG(" " << hUsn(rowIdx, colIdx, cIndex) << ",");
            }
        }
        channelNorm += clusterNorm;
    }

    NS_LOG_INFO("size of coefficient matrix (rows, columns, clusters) = ("
                << hUsn.GetNumRows() << ", " << hUsn.GetNumCols() << ", " << hUsn.GetNumPages()
                << ")" << "Channel norm " << channelNorm);
    channelMatrix->m_channel = hUsn;
    return channelMatrix;
}

} // namespace ns3
