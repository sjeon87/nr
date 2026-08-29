// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - multi-carrier BS power consumption

/**
 * @file gsoc-nr-energy-ca-example.cc
 * @ingroup examples
 *
 * @brief NR energy framework on a carrier-aggregated gNB (TR 38.864 Section 5.1).
 *
 * The companion gsoc-nr-energy-example runs one carrier with one bandwidth part.
 * This one builds several component carriers, each split into several bandwidth
 * parts, because the two are charged very differently and the difference is the
 * whole point of the multi-carrier support:
 *
 *   - bandwidth parts of ONE carrier -> their occupancy is aggregated and the
 *     TR 38.864 formula is evaluated once. P_DL carries the static baseline P3
 *     and the load-independent share A, so evaluating per bandwidth part and
 *     then combining would count both once per part.
 *
 *   - carriers -> their POWERS are summed, each keeping its own full P3, with
 *     each additional intra-band contiguous carrier scaled by 0.7:
 *     *"For multi-carrier, the total power consumption of BS is calculated as is
 *     the sum of the power consumption of each CC; for intra-band multi-carrier
 *     with contiguous CCs, the power consumption of each additional CC is scaled
 *     by 0.7."*
 *
 * All of that is handled by passing the operation bands to InstallGnb(). The
 * helper groups bandwidth parts into carriers, gives each carrier's model to an
 * NrGnbEnergyAggregator together with its band and frequency edges, and the
 * aggregator derives the weights itself. Nothing in this file decides what is
 * contiguous.
 *
 * Traffic is split across two carriers by mapping two QoS flow types to
 * bandwidth parts in different carriers, so the reported per-carrier energies
 * differ and the aggregation is actually exercised.
 *
 * Usage:
 *   ./ns3 run "gsoc-nr-energy-ca-example"
 *   ./ns3 run "gsoc-nr-energy-ca-example --numCc=3 --numBwpPerCc=1"
 *   ./ns3 run "gsoc-nr-energy-ca-example --numCc=1 --numBwpPerCc=2"
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/core-module.h"
#include "ns3/energy-source-container.h"
#include "ns3/internet-module.h"
#include "ns3/nr-energy-helper.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GsocNrEnergyCaExample");

int
main(int argc, char* argv[])
{
    uint16_t ueNumPergNb = 4;
    Time simTime = MilliSeconds(400);
    Time udpAppStartTime = MilliSeconds(100);
    uint16_t numerology = 1;         // 30 kHz SCS (TR 38.864 Set 1)
    double centralFrequency = 3.5e9; // FR1
    double bandwidth = 200e6;        // split across the carriers below
    double totalTxPower = 43;        // dBm, per bandwidth part
    uint32_t udpPacketSize = 1252;
    uint32_t lambda = 5000;  // packets/s per UE
    uint8_t numCc = 2;       // component carriers in the band
    uint8_t numBwpPerCc = 2; // bandwidth parts inside each carrier
    bool enableDrx = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("ueNumPergNb", "Number of UEs", ueNumPergNb);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerology", "BWP numerology", numerology);
    cmd.AddValue("lambda", "UDP packets per second per UE", lambda);
    cmd.AddValue("numCc", "Component carriers in the operation band", numCc);
    cmd.AddValue("numBwpPerCc", "Bandwidth parts per component carrier", numBwpPerCc);
    cmd.AddValue("enableDrx", "Install a C-DRX model on each UE", enableDrx);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(numCc < 1 || numBwpPerCc < 1, "numCc and numBwpPerCc must be at least 1");

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ---------------------------------------------------------------------
    // 1. Scenario. Nothing energy-specific.
    // ---------------------------------------------------------------------
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(1);
    gridScenario.SetHorizontalBsDistance(10.0);
    gridScenario.SetBsHeight(10.0);
    gridScenario.SetUtHeight(1.5);
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(1);
    gridScenario.SetUtNumber(ueNumPergNb);
    gridScenario.SetScenarioHeight(3);
    gridScenario.SetScenarioLength(3);
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    NodeContainer ueContainer = gridScenario.GetUserTerminals();
    NodeContainer gnbContainer = gridScenario.GetBaseStations();

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // ONE operation band holding numCc contiguous carriers, each split into
    // numBwpPerCc bandwidth parts. Contiguous is what earns the 0.7 on every
    // carrier after the first; a non-contiguous band would not.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCc);
    bandConf.m_numBwp = numBwpPerCc;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});

    // This flattening - band, then carriers, then bandwidth parts - is what
    // fixes the bandwidth part indices on the device. The energy helper replays
    // it to recover which bandwidth part belongs to which carrier, so the SAME
    // band object must be handed to both calls.
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
    const uint32_t totalBwps = allBwps.size();

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Put the two traffic types on bandwidth parts belonging to DIFFERENT
    // carriers where possible, so the per-carrier energies below actually
    // differ. With one carrier both land on it and the totals simply add.
    const uint32_t bwpForEmbb = 0;
    const uint32_t bwpForVoice = (numCc > 1) ? numBwpPerCc : (totalBwps - 1);

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpForEmbb));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpForEmbb));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpForVoice));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpForVoice));

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Every bandwidth part needs its numerology and power: the energy listeners
    // read the symbol period off each PHY as they attach.
    for (uint32_t bwp = 0; bwp < totalBwps; ++bwp)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(0), bwp)
            ->SetAttribute("Numerology", UintegerValue(numerology));
        NrHelper::GetGnbPhy(gnbNetDev.Get(0), bwp)
            ->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.0));
    InternetStackHelper internet;
    internet.Install(ueContainer);
    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // ---- downlink traffic: alternate UEs between the two flow types ----
    uint16_t dlPortEmbb = 1234;
    uint16_t dlPortVoice = 1235;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    serverApps.Add(UdpServerHelper(dlPortEmbb).Install(ueContainer));
    serverApps.Add(UdpServerHelper(dlPortVoice).Install(ueContainer));

    NrQosFlow embbFlow(NrQosFlow::NGBR_LOW_LAT_EMBB);
    NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE);

    for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    {
        const bool useVoice = (i % 2 == 1);
        const uint16_t port = useVoice ? dlPortVoice : dlPortEmbb;

        UdpClientHelper dlClient;
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda)));
        dlClient.SetAttribute(
            "Remote",
            AddressValue(addressUtils::ConvertToSocketAddress(ueIpIface.GetAddress(i), port)));
        clientApps.Add(dlClient.Install(remoteHost));

        Ptr<NrQosRule> rule = Create<NrQosRule>();
        NrQosRule::PacketFilter pf;
        pf.localPortStart = port;
        pf.localPortEnd = port;
        rule->Add(pf);
        nrHelper->ActivateDedicatedQosFlow(ueNetDev.Get(i), useVoice ? voiceFlow : embbFlow, rule);
    }

    Time trafficStopTime = udpAppStartTime + (simTime - udpAppStartTime) / 2;
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(trafficStopTime);

    // ---------------------------------------------------------------------
    // 2. Energy sources, exactly as in the single-carrier example.
    // ---------------------------------------------------------------------
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1e9));
    energy::EnergySourceContainer gnbSources = srcHelper.Install(gnbContainer);
    energy::EnergySourceContainer ueSources = srcHelper.Install(ueContainer);

    // ---------------------------------------------------------------------
    // 3./4. Install. The band is the only extra argument, and it is what turns
    //       "one model per device" into "one model per component carrier".
    // ---------------------------------------------------------------------
    NrEnergyHelper energyHelper;
    energyHelper.SetGnbEnergyModelAttribute("ReferenceTxPowerDbm", DoubleValue(totalTxPower));
    energyHelper.EnableDrx(enableDrx);

    energy::DeviceEnergyModelContainer gnbModels =
        energyHelper.InstallGnb(gnbNetDev, gnbSources, {band});
    energy::DeviceEnergyModelContainer ueModels = energyHelper.InstallUe(ueNetDev, ueSources);

    Simulator::Stop(simTime);
    Simulator::Run();

    // ---------------------------------------------------------------------
    // 5. Results. With more than one carrier the device model IS the
    //    aggregator, so the per-carrier breakdown is available through it.
    // ---------------------------------------------------------------------
    const double simSeconds = simTime.GetSeconds();
    uint64_t rxPackets = 0;
    for (uint32_t i = 0; i < serverApps.GetN(); ++i)
    {
        rxPackets += DynamicCast<UdpServer>(serverApps.Get(i))->GetReceived();
    }
    const double rxBits = static_cast<double>(rxPackets) * udpPacketSize * 8.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=========== NR Energy Example - Carrier Aggregation ===========\n";
    std::cout << "Band " << +band.m_bandId << ": " << +numCc << " CC x " << +numBwpPerCc
              << " BWP = " << totalBwps << " bandwidth parts, " << simSeconds << " s\n";
    std::cout << "eMBB traffic on BWP " << bwpForEmbb << ", voice on BWP " << bwpForVoice << "\n\n";

    const double deviceJ = gnbModels.Get(0)->GetTotalEnergyConsumption();
    Ptr<NrGnbEnergyAggregator> aggregator = DynamicCast<NrGnbEnergyAggregator>(gnbModels.Get(0));

    if (aggregator)
    {
        std::cout << "Per carrier (weight from TR 38.864 Section 5.1):\n";
        double weightedSum = 0.0;
        for (uint32_t cc = 0; cc < aggregator->GetNCarriers(); ++cc)
        {
            const double w = aggregator->GetCarrierWeight(cc);
            const double e = aggregator->GetCarrier(cc)->GetTotalEnergyJ();
            weightedSum += w * e;
            std::cout << "  CC" << cc << "  weight " << std::setprecision(2) << w
                      << std::setprecision(3) << "   own energy " << e << " J"
                      << "   contributes " << w * e << " J\n";
        }
        std::cout << "\n  sum of contributions : " << weightedSum << " J\n";
        std::cout << "  device total         : " << deviceJ << " J\n";
    }
    else
    {
        std::cout << "Single carrier: the device model is the carrier model.\n";
    }

    std::cout << "\ngNB total energy : " << deviceJ << " J\n";
    std::cout << "gNB average power: " << deviceJ / simSeconds << " W\n\n";

    double ueEnergyTotal = 0.0;
    for (uint32_t i = 0; i < ueModels.GetN(); ++i)
    {
        ueEnergyTotal += ueModels.Get(i)->GetTotalEnergyConsumption();
    }
    std::cout << "UE energy total  : " << ueEnergyTotal << " J\n";

    if (deviceJ > 0.0)
    {
        std::cout << "Network efficiency: " << (rxBits / deviceJ) / 1e6 << " Mbit/J\n";
    }
    std::cout << "===============================================================\n";

    Simulator::Destroy();
    return 0;
}
