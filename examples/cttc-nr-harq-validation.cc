// Copyright (c) 2026 Centre Tecnològic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Joseanne Viana <josi_ocroch@outlook.com>
//
// This example is a minimal, self-contained scenario to validate the HARQ
// behavior of the 5G-LENA NR module, and in particular the configurable-HARQ
// feature (enabling/disabling HARQ and limiting the number of retransmissions).
// - 1 gNB, 1 UE, 1 band/1 CC/1 BWP
// - fixed MCS
// - configurable distance + TxPower
// - configurable number of retransmissions
// - one DL UDP flow + FlowMonitor summary
//
// How it works:
//   A fixed MCS and a fixed distance/TxPower are used so the first-transmission
//   BLER is deterministic and reproducible (with a fixed RngSeed/RngRun). One DL
//   UDP flow is generated, and a FlowMonitor summary plus HARQ counters (average
//   retransmissions per TB, BLER before and after HARQ, fraction of TBs reaching
//   maxHarqReTx) are printed at the end, so the effect of HARQ can be observed
//   directly.
//
// Build/run:
//   ./ns3 run "cttc-nr-harq-validation --PrintHelp"
//
// Suggested validation sweep (fixed seed, reproducible results):
//   MCS 24 at -6 dBm sits on the HARQ decoding cliff, where increasing the
//   number of retransmissions has a clear, visible effect. Sweeping maxHarqReTx
//   shows HARQ recovering more packets up to the default of 3, then diminishing
//   (and eventually negative) returns as extra retransmissions consume slots that
//   would otherwise carry new data:
//
//   for b in 0 1 2 3 5; do
//     ./ns3 run "cttc-nr-harq-validation --enableHarq=1 --maxHarqReTx=$b
//       --totalTxPower=-6 --ueDistance=20 --mcs=24 --lambda=2000
//       --RngSeed=1 --RngRun=1"
//   done
//
//   Expected Rx packets: 0 (b=0), 450 (b=1), 719 (b=2), 807 (b=3), 552 (b=5).

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcHarqValidation");

int
main(int argc, char* argv[])
{
    // ---- CLI parameters ----
    bool enableHarq = true;
    double totalTxPower = -6.0; // dBm
    double ueDistance = 20.0;   // meters
    uint32_t mcs = 24;          // fixed MCS (0..28 typical)
    uint16_t numerology = 3;
    double centralFrequency = 28e9; // Hz
    double bandwidth = 50e6;        // Hz

    uint32_t packetSize = 1200; // bytes
    uint32_t lambda = 2000;     // packets/s
    Time simTime = MilliSeconds(1000);
    Time appStart = MilliSeconds(200);
    uint32_t maxHarqReTx = 3;
    bool logging = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("enableHarq", "Enable HARQ (true/false)", enableHarq);
    cmd.AddValue("totalTxPower", "Total gNB TxPower (dBm)", totalTxPower);
    cmd.AddValue("ueDistance", "Distance between gNB and UE (m)", ueDistance);
    cmd.AddValue("mcs", "Fixed MCS used when FixedMcsDl/Ul=true", mcs);
    cmd.AddValue("numerology", "Numerology (mu)", numerology);
    cmd.AddValue("centralFrequency", "Carrier frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "Bandwidth (Hz)", bandwidth);
    cmd.AddValue("packetSize", "UDP packet size (bytes)", packetSize);
    cmd.AddValue("lambda", "UDP packets per second", lambda);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("appStart", "Application start time", appStart);
    cmd.AddValue("logging", "Enable a few logs from code", logging);
    cmd.AddValue(
        "maxHarqReTx",
        "Maximum number of HARQ retransmissions (0 = no retransmissions, 3 = default NR behavior)",
        maxHarqReTx);
    cmd.Parse(argc, argv);

    if (logging)
    {
        LogComponentEnable("CttcHarqValidation", LOG_LEVEL_INFO);
    }

    // ---- Nodes ----
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    // Positions: gNB at (0,0,10), UE at (d,0,1.5)
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 10.0));
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(ueDistance, 0.0, 1.5));

    // ---- NR helpers ----
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(bfHelper);

    // Channel setup (3GPP 38.901 style)
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // One band / one CC / one BWP
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Beamforming
    bfHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Zero core delay (so radio dominates)
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
    nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
    nrHelper->SetGnbDlAmcAttribute("ErrorModelType", StringValue("ns3::NrEesmIrT1"));
    nrHelper->SetGnbUlAmcAttribute("ErrorModelType", StringValue("ns3::NrEesmIrT1"));

    nrHelper->SetHarqEnabled(enableHarq);
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaRR"));
    nrHelper->SetSchedulerAttribute("MaxHarqReTx", UintegerValue(maxHarqReTx));
    // Fixed MCS via scheduler (THIS is the correct place)
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(mcs));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(mcs));

    // Set RLC reordering timer to 0 to eliminate RLC reordering delay,
    // so that FlowMonitor delay reflects only radio and HARQ timing.
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(0)));

    // Antennas (keep simple)
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));

    // Install devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Per-BWP config
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerology));

    // TxPower: totalTxPower (single band -> no split)
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("TxPower", DoubleValue(totalTxPower));

    // ---- Internet + EPC ----
    auto [remoteHost, remoteHostAddr] = epcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    InternetStackHelper internet;
    internet.Install(ueNodes);

    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // Attach UE to gNB
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // ---- Traffic (DL UDP) ----
    uint16_t dlPort = 1234;

    // Create a QoS flow and a rule matching the DL UDP port
    NrQosFlow flow(NrQosFlow::NGBR_LOW_LAT_EMBB);

    // Create a default QoS rule (match-all traffic)
    Ptr<NrQosRule> rule = Create<NrQosRule>();

    nrHelper->ActivateDedicatedQosFlow(ueDevs.Get(0), flow, rule);

    UdpServerHelper server(dlPort);
    ApplicationContainer serverApps = server.Install(ueNodes.Get(0));

    UdpClientHelper client;
    client.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIfaces.GetAddress(0), dlPort)));
    client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0 / static_cast<double>(lambda))));

    ApplicationContainer clientApps = client.Install(remoteHost);

    serverApps.Start(appStart);
    clientApps.Start(appStart);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // ---- FlowMonitor ----
    FlowMonitorHelper flowmonHelper;
    NodeContainer endpoints;
    endpoints.Add(remoteHost);
    endpoints.Add(ueNodes);
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(endpoints);

    // ---- Run ----
    nrHelper->EnableTraces();
    Simulator::Stop(simTime);
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    auto stats = monitor->GetFlowStats();
    std::cout << "Number of flows: " << stats.size() << std::endl;

    double flowDuration = (simTime - appStart).GetSeconds();
    double sumThr = 0.0;
    double sumDelayMs = 0.0;
    uint32_t counted = 0;

    std::cout.setf(std::ios_base::fixed);
    std::cout << "==== HARQ sanity results ====\n";
    std::cout << "enableHarq=" << (enableHarq ? "1" : "0") << "  "
              << "TxPower=" << totalTxPower << " dBm  "
              << "distance=" << ueDistance << " m  "
              << "mcs=" << mcs << "  "
              << "numerology=" << numerology << "  "
              << "maxHarqReTx=" << maxHarqReTx << "\n\n";

    for (const auto& kv : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        const auto& st = kv.second;

        std::cout << "Flow " << kv.first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                  << t.destinationAddress << ":" << t.destinationPort << ") proto UDP\n";
        std::cout << "  Tx Packets: " << st.txPackets << "\n";
        std::cout << "  Rx Packets: " << st.rxPackets << "\n";
        std::cout << "  Lost Packets: " << st.lostPackets << "\n";

        double thrMbps = (st.rxBytes * 8.0) / flowDuration / 1e6;
        std::cout << "  Throughput: " << thrMbps << " Mbps\n";

        if (st.rxPackets > 0)
        {
            double meanDelayMs = 1000.0 * st.delaySum.GetSeconds() / st.rxPackets;
            std::cout << "  Mean delay: " << meanDelayMs << " ms\n";
            sumThr += thrMbps;
            sumDelayMs += meanDelayMs;
            counted++;
        }
        else
        {
            std::cout << "  Mean delay: 0 ms\n";
        }
        std::cout << "\n";
    }

    if (counted > 0)
    {
        std::cout << "Mean flow throughput: " << (sumThr / counted) << " Mbps\n";
        std::cout << "Mean flow delay: " << (sumDelayMs / counted) << " ms\n";
    }

    Ptr<NrMacSchedulerNs3> sched =
        DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(gnbDevs.Get(0), 0));

    std::cout << "\n===== HARQ Detailed Stats =====\n";
    if (enableHarq)
    {
        std::cout << "Avg Retx per TB: " << sched->GetAvgRetxPerTb() << "\n";
        std::cout << "HARQ overhead: " << sched->GetHarqOverhead() * 100 << " %\n";
    }
    else
    {
        std::cout << "Avg Retx per TB: N/A (HARQ disabled)\n";
        std::cout << "HARQ overhead: N/A (HARQ disabled)\n";
    }

    double blerBefore = 0.0;
    double blerAfter = 0.0;

    if (enableHarq)
    {
        blerBefore = sched->GetBlerBeforeHarq();
        blerAfter = sched->GetBlerAfterHarq();
    }
    else
    {
        // Compute BLER from FlowMonitor stats
        uint64_t txPackets = 0;
        uint64_t rxPackets = 0;

        for (const auto& flow : stats)
        {
            txPackets += flow.second.txPackets;
            rxPackets += flow.second.rxPackets;
        }

        if (txPackets > 0)
        {
            blerBefore = static_cast<double>(txPackets - rxPackets) / txPackets;
            blerAfter = blerBefore; // no HARQ → same value
        }
    }

    if (!enableHarq)
    {
        std::cout << "(HARQ disabled → no retransmissions, BLER from FlowMonitor)\n";
    }

    std::cout << "BLER before HARQ: " << blerBefore << "\n";
    std::cout << "BLER after HARQ: " << blerAfter << "\n";

    if (enableHarq)
    {
        std::cout << "% TB reaching maxHarqReTx: " << sched->GetTbReachedMaxRatio() * 100 << " %\n";
    }

    Simulator::Destroy();
    return 0;
}
