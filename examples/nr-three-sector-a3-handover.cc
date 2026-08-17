// Copyright (c) 2026
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file nr-three-sector-a3-handover-style.cc
 * @ingroup examples
 * @brief NR three-sector handover example with rotating UEs.
 *
 * This example creates one NR site with three sectors and a set of UEs moving
 * on circular trajectories around the site. The built-in
 * NrA3RsrpHandoverAlgorithm is used to trigger handovers between sectors.
 *
 * The example uses standard 5G-LENA helpers for deployment, EPC setup,
 * attachment, traffic generation, and tracing. It reports:
 *  - UE/gNB connection establishment
 *  - UE RRC state transitions (only HO/RLF-relevant ones by default)
 *  - PHY sync indications
 *  - handover start / end
 *  - radio link failures
 *
 * Example:
 * @code
 * ./ns3 run "scratch/nr-three-sector-a3-handover-example"
 * @endcode
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/netanim-module.h"

#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrThreeSectorA3HandoverStyle");

namespace
{

constexpr uint32_t g_numSectors = 3;
constexpr double g_pi = 3.14159265358979323846;

struct UeStats
{
    uint32_t outOfSyncEvents = 0;
    uint32_t inSyncEvents = 0;
    uint32_t rlfCount = 0;

    uint32_t ueHandoverStartCount = 0;
    uint32_t ueHandoverEndOkCount = 0;
    uint32_t ueHandoverEndErrorCount = 0;

    uint32_t gnbHandoverStartCount = 0;
    uint32_t gnbHandoverEndOkCount = 0;

    uint16_t lastCellId = 0;
    uint16_t pendingSourceCellId = 0;
    uint16_t pendingTargetCellId = 0;
    uint16_t lastRnti = 0;
};

std::map<uint64_t, UeStats> g_ueStats;

std::string
UeRrcStateToString(NrUeRrc::State s)
{
    switch (s)
    {
    case NrUeRrc::IDLE_START:
        return "IDLE_START";
    case NrUeRrc::IDLE_CELL_SEARCH:
        return "IDLE_CELL_SEARCH";
    case NrUeRrc::IDLE_WAIT_MIB_SIB1:
        return "IDLE_WAIT_MIB_SIB1";
    case NrUeRrc::IDLE_WAIT_MIB:
        return "IDLE_WAIT_MIB";
    case NrUeRrc::IDLE_WAIT_SIB1:
        return "IDLE_WAIT_SIB1";
    case NrUeRrc::IDLE_CAMPED_NORMALLY:
        return "IDLE_CAMPED_NORMALLY";
    case NrUeRrc::IDLE_WAIT_SIB2:
        return "IDLE_WAIT_SIB2";
    case NrUeRrc::IDLE_RANDOM_ACCESS:
        return "IDLE_RANDOM_ACCESS";
    case NrUeRrc::IDLE_CONNECTING:
        return "IDLE_CONNECTING";
    case NrUeRrc::CONNECTED_NORMALLY:
        return "CONNECTED_NORMALLY";
    case NrUeRrc::CONNECTED_HANDOVER:
        return "CONNECTED_HANDOVER";
    case NrUeRrc::CONNECTED_PHY_PROBLEM:
        return "CONNECTED_PHY_PROBLEM";
    case NrUeRrc::CONNECTED_REESTABLISHING:
        return "CONNECTED_REESTABLISHING";
    default:
        return "UNKNOWN_STATE";
    }
}

Vector
GetUePosition(uint64_t imsi)
{
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;

        for (uint32_t i = 0; i < node->GetNDevices(); ++i)
        {
            Ptr<NrUeNetDevice> ueDevice = node->GetDevice(i)->GetObject<NrUeNetDevice>();
            if (ueDevice != nullptr && ueDevice->GetImsi() == imsi)
            {
                Ptr<MobilityModel> mm = node->GetObject<MobilityModel>();
                if (mm != nullptr)
                {
                    return mm->GetPosition();
                }
            }
        }
    }

    return Vector(0.0, 0.0, 0.0);
}

void
PrintUePosition(uint64_t imsi)
{
    Vector position = GetUePosition(imsi);
    std::cout << "    UE IMSI=" << imsi << " position=(" << position.x << ", " << position.y
              << ", " << position.z << ")" << std::endl;
}

bool
IsInterestingUeTransition(NrUeRrc::State oldState, NrUeRrc::State newState)
{
    return oldState == NrUeRrc::CONNECTED_HANDOVER ||
           newState == NrUeRrc::CONNECTED_HANDOVER ||
           oldState == NrUeRrc::CONNECTED_PHY_PROBLEM ||
           newState == NrUeRrc::CONNECTED_PHY_PROBLEM ||
           oldState == NrUeRrc::CONNECTED_REESTABLISHING ||
           newState == NrUeRrc::CONNECTED_REESTABLISHING;
}

std::string
TransitionCategory(NrUeRrc::State oldState, NrUeRrc::State newState)
{
    if (oldState == NrUeRrc::CONNECTED_HANDOVER || newState == NrUeRrc::CONNECTED_HANDOVER)
    {
        return "HANDOVER";
    }

    if (oldState == NrUeRrc::CONNECTED_PHY_PROBLEM ||
        newState == NrUeRrc::CONNECTED_PHY_PROBLEM ||
        oldState == NrUeRrc::CONNECTED_REESTABLISHING ||
        newState == NrUeRrc::CONNECTED_REESTABLISHING)
    {
        return "RLF/RECOVERY";
    }

    return "ATTACH/IDLE";
}

double
NormalizeAngle(double a)
{
    while (a >= g_pi)
    {
        a -= 2.0 * g_pi;
    }
    while (a < -g_pi)
    {
        a += 2.0 * g_pi;
    }
    return a;
}











                             
void NotifyConnectionEstablishedGnb(std::string context,
                                    uint64_t imsi,
                                    uint16_t cellId,
                                    uint16_t rnti)
{
    Vector p = GetUePosition(imsi);
    std::cout << Simulator::Now().GetSeconds() << "s [GNB-CONNECTED]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
}

void NotifyConnectionEstablishedUe(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellId,
                                   uint16_t rnti)
{
    Vector p = GetUePosition(imsi);
    std::cout << Simulator::Now().GetSeconds() << "s [UE-CONNECTED]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyUeStateTransition(std::string context,
                             uint64_t imsi,
                             uint16_t cellId,
                             uint16_t rnti,
                             NrUeRrc::State oldState,
                             NrUeRrc::State newState)
{
    // Keep the output focused on handover / RLF related state changes.
    if (!IsInterestingUeTransition(oldState, newState))
    {
        return;
    }

    Vector p = GetUePosition(imsi);

    std::cout << Simulator::Now().GetSeconds() << "s [" << TransitionCategory(oldState, newState)
              << "] UE state transition"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " state=" << UeRrcStateToString(oldState) << " -> "
              << UeRrcStateToString(newState) << " pos=(" << p.x << ", " << p.y << ", " << p.z
              << ")" << std::endl;
}

void NotifyConnectionReleaseGnb(std::string context,
                                uint64_t imsi,
                                uint16_t cellId,
                                uint16_t rnti)


{
    Vector p = GetUePosition(imsi);
    std::cout << Simulator::Now().GetSeconds() << "s [GNB-RELEASE]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
}
void NotifyPhySyncDetection(std::string context,
                            uint64_t imsi,
                            uint16_t rnti,
                            uint16_t cellId,
                            std::string type,
                            uint8_t count)
{
    Vector p = GetUePosition(imsi);

    std::cout << Simulator::Now().GetSeconds() << "s [PHY-SYNC]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " type=" << type << " count=" << static_cast<uint32_t>(count)
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

    if (type == "Notify out of sync")
    {
        g_ueStats[imsi].outOfSyncEvents++;
    }
    else if (type == "Notify in sync")
    {
        g_ueStats[imsi].inSyncEvents++;
    }

    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyRadioLinkFailure(std::string context,
                            uint64_t imsi,
                            uint16_t cellId,
                            uint16_t rnti)


{
    std::cout << '\n';
    std::cout << Simulator::Now().GetSeconds() << "s [RLF]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti << std::endl;
    PrintUePosition(imsi);
    std::cout << std::endl;

    g_ueStats[imsi].rlfCount++;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyGnbHandoverStart(std::string context,
                            uint64_t imsi,
                            uint16_t cellId,
                            uint16_t rnti,
                            uint16_t targetCellId)


{
    Vector p = GetUePosition(imsi);

    std::cout << '\n';
    std::cout << Simulator::Now().GetSeconds() << "s [GNB-HO-START]"
              << " IMSI=" << imsi << " RNTI=" << rnti << " fromCell=" << cellId
              << " toCell=" << targetCellId << " pos=(" << p.x << ", " << p.y << ", " << p.z
              << ")" << std::endl;

    g_ueStats[imsi].gnbHandoverStartCount++;
    g_ueStats[imsi].pendingSourceCellId = cellId;
    g_ueStats[imsi].pendingTargetCellId = targetCellId;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyGnbHandoverEndOk(std::string context,
                            uint64_t imsi,
                            uint16_t cellId,
                            uint16_t rnti)
{
    Vector p = GetUePosition(imsi);

    std::cout << Simulator::Now().GetSeconds() << "s [GNB-HO-END-OK]"
              << " IMSI=" << imsi << " RNTI=" << rnti
              << " " << g_ueStats[imsi].pendingSourceCellId << " -> "
              << g_ueStats[imsi].pendingTargetCellId << " confirmedCell=" << cellId
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

    g_ueStats[imsi].gnbHandoverEndOkCount++;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyUeHandoverStart(std::string context,
                           uint64_t imsi,
                           uint16_t cellId,
                           uint16_t rnti,
                           uint16_t targetCellId)


{
    Vector p = GetUePosition(imsi);

    std::cout << '\n';
    std::cout << Simulator::Now().GetSeconds() << "s [UE-HO-START]"
              << " IMSI=" << imsi << " RNTI=" << rnti << " fromCell=" << cellId
              << " toCell=" << targetCellId << " pos=(" << p.x << ", " << p.y << ", " << p.z
              << ")" << std::endl;

    g_ueStats[imsi].ueHandoverStartCount++;
    g_ueStats[imsi].pendingSourceCellId = cellId;
    g_ueStats[imsi].pendingTargetCellId = targetCellId;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyUeHandoverEndOk(std::string context,
                           uint64_t imsi,
                           uint16_t cellId,
                           uint16_t rnti)


{
    Vector p = GetUePosition(imsi);

    std::cout << Simulator::Now().GetSeconds() << "s [UE-HO-END-OK]"
              << " IMSI=" << imsi << " RNTI=" << rnti
              << " " << g_ueStats[imsi].pendingSourceCellId << " -> "
              << g_ueStats[imsi].pendingTargetCellId << " confirmedCell=" << cellId
              << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

    g_ueStats[imsi].ueHandoverEndOkCount++;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].pendingSourceCellId = 0;
    g_ueStats[imsi].pendingTargetCellId = 0;
    g_ueStats[imsi].lastRnti = rnti;
}

void NotifyUeHandoverEndError(std::string context,
                              uint64_t imsi,
                              uint16_t cellId,
                              uint16_t rnti)


{
    Vector p = GetUePosition(imsi);

    std::cout << Simulator::Now().GetSeconds() << "s [UE-HO-END-ERROR]"
              << " IMSI=" << imsi << " cellId=" << cellId << " RNTI=" << rnti
              << " pending=" << g_ueStats[imsi].pendingSourceCellId << " -> "
              << g_ueStats[imsi].pendingTargetCellId << " pos=(" << p.x << ", " << p.y << ", "
              << p.z << ")" << std::endl;

    g_ueStats[imsi].ueHandoverEndErrorCount++;
    g_ueStats[imsi].lastCellId = cellId;
    g_ueStats[imsi].pendingSourceCellId = 0;
    g_ueStats[imsi].pendingTargetCellId = 0;
    g_ueStats[imsi].lastRnti = rnti;
}

void
NotifyMeasurementReport(std::string context,
                        uint64_t imsi,
                        uint16_t cellId,
                        uint16_t rnti,
                        NrRrcSap::MeasurementReport report)
{
    std::cout << Simulator::Now().GetSeconds()
              << "s [MEAS-REPORT]"
              << " IMSI=" << imsi
              << " servingCell=" << cellId
              << " RNTI=" << rnti
              << " measId=" << +report.measResults.measId
              << " pcellRsrp=" << +report.measResults.measResultPCell.rsrpResult
              << " haveNeigh=" << report.measResults.haveMeasResultNeighCells;

    if (report.measResults.haveMeasResultNeighCells)
    {
        for (const auto& n : report.measResults.measResultListEutra)
        {
            std::cout << " [nbrCell=" << n.physCellId;
            if (n.haveRsrpResult)
            {
                std::cout << " rsrp=" << +n.rsrpResult;
            }
            std::cout << "]";
        }
    }
    std::cout << std::endl;
}

void
UpdateOrbit(Ptr<Node> ueNode,
            const Vector& center,
            double radius,
            double angularSpeed,
            double phase,
            double height,
            Time stopTime,
            Time step)
{
    if (Simulator::Now() >= stopTime)
    {
        return;
    }

    double t = Simulator::Now().GetSeconds();
    double angle = phase + angularSpeed * t;
    Vector position(center.x + radius * std::cos(angle),
                    center.y + radius * std::sin(angle),
                    height);

    ueNode->GetObject<MobilityModel>()->SetPosition(position);

    Simulator::Schedule(step,
                        &UpdateOrbit,
                        ueNode,
                        center,
                        radius,
                        angularSpeed,
                        phase,
                        height,
                        stopTime,
                        step);
}

void
PrintSummary()
{
    std::cout << "\n========== Summary ==========\n";
    for (const auto& [imsi, stats] : g_ueStats)
    {
        std::cout << "IMSI=" << imsi << " lastCell=" << stats.lastCellId
                  << " lastRnti=" << stats.lastRnti
                  << " ueHoStart=" << stats.ueHandoverStartCount
                  << " ueHoEndOk=" << stats.ueHandoverEndOkCount
                  << " ueHoEndError=" << stats.ueHandoverEndErrorCount
                  << " gnbHoStart=" << stats.gnbHandoverStartCount
                  << " gnbHoEndOk=" << stats.gnbHandoverEndOkCount
                  << " outOfSync=" << stats.outOfSyncEvents
                  << " inSync=" << stats.inSyncEvents
                  << " rlfCount=" << stats.rlfCount << std::endl;
    }
    std::cout << "=============================\n" << std::endl;
}

TypeId
GetBeamformingMethodTypeId(const std::string& beamformingMethod)
{
    if (beamformingMethod == "dir-dir")
    {
        return DirectPathBeamforming::GetTypeId();
    }

    if (beamformingMethod == "dir-omni")
    {
        return DirectPathQuasiOmniBeamforming::GetTypeId();
    }

    if (beamformingMethod == "omni-dir")
    {
        return QuasiOmniDirectPathBeamforming::GetTypeId();
    }

    if (beamformingMethod == "search-omni")
    {
        return CellScanQuasiOmniBeamforming::GetTypeId();
    }

    NS_FATAL_ERROR("Unsupported beamforming method: " << beamformingMethod);
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t ueNum = 2;
    double simTime = 20.0;

    double gnbHeight = 25.0;
    double ueHeight = 1.5;
    double orbitRadius = 150.0;
    double angularSpeed = 0.25; // rad/s

    double frequency = 2e9;
    double bandwidth = 20e6;
    uint16_t numerology = 0;
    double txPower = 20.0;

    std::string scenario = "UMi";
    std::string condition = "Default";
    std::string beamformingMethod = "dir-dir";
    std::string scheduler = "ns3::NrMacSchedulerTdmaPF";

    bool isoUe = true;
    bool isoGnb = false;
    uint32_t numRowsUe = 1;
    uint32_t numColumnsUe = 1;
    uint32_t numRowsGnb = 4;
    uint32_t numColumnsGnb = 4;

    bool useIdealRrc = true;
    bool enableRem = false;
    bool logging = true;

    double a3HysteresisDb = 0;
    Time a3TimeToTrigger = MilliSeconds(0);

    uint16_t n310 = 1;
    uint16_t n311 = 1;
    Time t310 = Seconds(0.0);

    std::string simTag = "";
    double xMin = -1000.0;
    double xMax = 1000.0;
    uint16_t xRes = 80;
    double yMin = -1000.0;
    double yMax = 1000.0;
    uint16_t yRes = 80;
    double z = 1.5;

    CommandLine cmd(__FILE__);
    cmd.AddValue("ueNum", "Total number of UEs", ueNum);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("orbitRadius", "UE orbit radius in meters", orbitRadius);
    cmd.AddValue("angularSpeed", "UE angular speed in rad/s", angularSpeed);
    cmd.AddValue("frequency", "Carrier frequency in Hz", frequency);
    cmd.AddValue("bandwidth", "Bandwidth in Hz", bandwidth);
    cmd.AddValue("numerology", "Numerology", numerology);
    cmd.AddValue("txPower", "gNB transmit power in dBm", txPower);
    cmd.AddValue("scenario", "3GPP scenario, e.g. UMa", scenario);
    cmd.AddValue("condition", "Channel condition", condition);
    cmd.AddValue("beamformingMethod",
                 "Beamforming method: dir-dir, dir-omni, omni-dir, search-omni",
                 beamformingMethod);
    cmd.AddValue("scheduler", "Scheduler type id", scheduler);
    cmd.AddValue("useIdealRrc", "Use ideal RRC", useIdealRrc);
    cmd.AddValue("a3HysteresisDb", "A3 hysteresis in dB", a3HysteresisDb);
    cmd.AddValue("a3TimeToTrigger", "A3 time to trigger", a3TimeToTrigger);
    cmd.AddValue("n310", "Number of out-of-sync indications before T310 starts", n310);
    cmd.AddValue("n311", "Number of in-sync indications", n311);
    cmd.AddValue("t310", "T310 timer", t310);
    cmd.AddValue("enableRem", "Enable optional REM output", enableRem);
    cmd.AddValue("simTag", "REM output tag", simTag);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(ueNum == 0, "ueNum must be greater than zero");

    if (logging)
    {
        LogComponentEnable("NrThreeSectorA3HandoverStyle", LOG_LEVEL_INFO);
        LogComponentEnable("NrA3RsrpHandoverAlgorithm", LOG_LEVEL_ALL);
        
       
    }

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(n310));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(n311));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(t310));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();

    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(useIdealRrc));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(a3HysteresisDb));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(a3TimeToTrigger));

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(GetBeamformingMethodTypeId(beamformingMethod)));

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    OperationBandInfo band;

    CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, 1);
    bandConf.m_numBwp = 1;
    band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                   TimeValue(MilliSeconds(10)));
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                 TimeValue(MilliSeconds(100)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});

    allBwps = CcBwpCreator::GetAllBwps({band});
    //(void)totalBandwdith;

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(numRowsUe));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(numColumnsUe));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(numRowsGnb));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(numColumnsGnb));

    if (isoUe)
    {
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObject<IsotropicAntennaModel>()));
    }
    else
    {
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObject<ThreeGppAntennaModel>()));
    }

    if (isoGnb)
    {
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                         PointerValue(CreateObject<IsotropicAntennaModel>()));
    }
    else
    {
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                         PointerValue(CreateObject<ThreeGppAntennaModel>()));
    }

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(g_numSectors);
    ueNodes.Create(ueNum);

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < g_numSectors; ++i)
    {
        gnbPositionAlloc->Add(Vector(0.0, 0.0, gnbHeight));
    }
    /*gnbPositionAlloc->Add(Vector(-600.0, 0.0, gnbHeight));
    gnbPositionAlloc->Add(Vector( 150.0, 0.0, gnbHeight));
    gnbPositionAlloc->Add(Vector( 0.0,  900.0, gnbHeight));*/

    std::vector<double> uePhases(ueNum);
    Vector orbitCenter(0.0, 0.0, ueHeight);
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < ueNum; ++i)
    {
        double phase = 2.0 * g_pi * static_cast<double>(i) / static_cast<double>(ueNum);
        uePhases[i] = phase;
        uePositionAlloc->Add(Vector(orbitCenter.x + orbitRadius * std::cos(phase),
                                    orbitCenter.y + orbitRadius * std::sin(phase),
                                    ueHeight));
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(gnbPositionAlloc);
    mobility.Install(gnbNodes);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Keep sector bearings within [-pi, pi).
    std::array<double, g_numSectors> sectorBearings = {
        0.0,
        2.0 * g_pi / 3.0,
        -2.0 * g_pi / 3.0};

    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0);
        gnbPhy->SetTxPower(txPower);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));

        Ptr<UniformPlanarArray> antennaArray =
            DynamicCast<UniformPlanarArray>(gnbPhy->GetSpectrumPhy()->GetAntenna());
        if (antennaArray != nullptr)
        {
            antennaArray->SetAttribute("BearingAngle",
                                       DoubleValue(NormalizeAngle(sectorBearings.at(i))));
        }
    }

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(gnbNetDev, stream);
    stream += nrHelper->AssignStreams(ueNetDev, stream);

    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));
    (void)remoteHostIpv4Address;

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        UdpServerHelper dlServer(dlPort + u);
        serverApps.Add(dlServer.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort + u);
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1024));
        clientApps.Add(dlClient.Install(remoteHost));
    }
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    nrHelper->AddX2Interface(gnbNodes);
    

    // Initial attachment to sector 0. A3 handover should later move UEs
    // across the three collocated sectors as their trajectories evolve.
    /*for (uint32_t u = 0; u < ueNetDev.GetN(); ++u)
    {
    //    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
        nrHelper->AttachToGnb(ueNetDev.Get(u), gnbNetDev.Get(0));
    }*/
    //nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
    nrHelper->AttachToMaxRsrpGnb(ueNetDev, gnbNetDev);

    serverApps.Start(Seconds(0.2));
    clientApps.Start(Seconds(0.2));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime - 0.1));

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/ConnectionEstablished",
                MakeCallback(&NotifyConnectionEstablishedGnb));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
                MakeCallback(&NotifyConnectionEstablishedUe));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/StateTransition",
                MakeCallback(&NotifyUeStateTransition));
Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/NotifyConnectionRelease",
                MakeCallback(&NotifyConnectionReleaseGnb));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/PhySyncDetection",
                MakeCallback(&NotifyPhySyncDetection));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
                MakeCallback(&NotifyRadioLinkFailure));
Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                MakeCallback(&NotifyGnbHandoverStart));
Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                MakeCallback(&NotifyGnbHandoverEndOk));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                MakeCallback(&NotifyUeHandoverStart));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                MakeCallback(&NotifyUeHandoverEndOk));
Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndError",
                MakeCallback(&NotifyUeHandoverEndError));
Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/RecvMeasurementReport",
                MakeCallback(&NotifyMeasurementReport));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Simulator::Schedule(Seconds(0.5),
                            &UpdateOrbit,
                            ueNodes.Get(u),
                            orbitCenter,
                            orbitRadius,
                            angularSpeed,
                            uePhases[u],
                            ueHeight,
                            Seconds(simTime),
                            MilliSeconds(50));
    }

    if (enableRem)
    {
        Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
        remHelper->SetMinX(xMin);
        remHelper->SetMaxX(xMax);
        remHelper->SetResX(xRes);
        remHelper->SetMinY(yMin);
        remHelper->SetMaxY(yMax);
        remHelper->SetResY(yRes);
        remHelper->SetZ(z);
        remHelper->SetSimTag(simTag);
        remHelper->SetRemMode(NrRadioEnvironmentMapHelper::COVERAGE_AREA);
        remHelper->CreateRem(gnbNetDev, ueNetDev.Get(0), 0);
    }
    AnimationInterface anim("nr-three-sector-a3.xml");
    anim.SetMobilityPollInterval(MilliSeconds(50));
    anim.SetStartTime(Seconds(0.0));
    anim.SetStopTime(Seconds(simTime));
    anim.SkipPacketTracing();

    Simulator::Schedule(Seconds(simTime - 0.01), &PrintSummary);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}