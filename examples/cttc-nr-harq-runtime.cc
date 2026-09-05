// Copyright (c) 2026 Centre Tecnològic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Joseanne Viana <josi_ocroch@outlook.com>
//
// This example is a minimal, self-contained scenario that demonstrates enabling
// and disabling HARQ at runtime in the 5G-LENA NR module.
// - 1 gNB, 1 UE, 1 band/1 CC/1 BWP
// - fixed MCS
// - configurable distance + TxPower
// - one DL UDP flow + per-window HARQ statistics
//
// How it works:
//   HARQ starts enabled and is toggled during the simulation by scheduling
//   NrGnbRrc::SetEnableHarq() on the live gNB RRC: it is disabled at 400 ms and
//   re-enabled at 700 ms, splitting the 1 s run into on/off/on windows. Toggling
//   through the gNB RRC triggers an RRC reconfiguration of the connected UE over
//   the air (TS 38.331). Per-window HARQ statistics (received packets, average
//   retransmissions per TB, BLER) are printed so the effect of enabling and
//   disabling HARQ at runtime can be observed directly.
//
// Build/run:
//   ./ns3 run "cttc-nr-harq-runtime --PrintHelp"
//
// Suggested command (fixed seed, reproducible results):
//   MCS 26 at -6 dBm places the link where the first transmission fails but HARQ
//   recovers it, so disabling HARQ mid-run clearly stops delivery and re-enabling
//   restores it. Note the offered load must stay moderate (lambda ~2000): a much
//   higher load saturates the channel and no packets get through even with HARQ on.
//
//   ./ns3 run "cttc-nr-harq-runtime --totalTxPower=-6 --ueDistance=20 --mcs=26
//     --maxHarqReTx=3 --lambda=2000 --RngSeed=1 --RngRun=1"
//
//   Expected per-window Rx: ~388 (on), 0 (off), ~166 (on).

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcHarqRuntime");

struct WindowStats
{
    uint64_t rxPackets{0};
    uint64_t rxBytes{0};
    NrMacSchedulerNs3::HarqCounters harq{};
};

static void
TakeSnapshot(Ptr<FlowMonitor> monitor, Ptr<NrMacSchedulerNs3> sched, WindowStats* out)
{
    monitor->CheckForLostPackets();
    uint64_t rx = 0;
    uint64_t bytes = 0;
    for (const auto& f : monitor->GetFlowStats())
    {
        rx += f.second.rxPackets;
        bytes += f.second.rxBytes;
    }
    out->rxPackets = rx;
    out->rxBytes = bytes;
    out->harq = sched->GetHarqCounters();
}

int
main(int argc, char* argv[])
{
    // ---- CLI parameters ----
    bool enableHarq = true;
    double totalTxPower = -6.0; // dBm
    double ueDistance = 20.0;   // meters
    uint32_t mcs = 26;          // fixed MCS (0..28 typical)
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
        LogComponentEnable("CttcHarqRuntime", LOG_LEVEL_INFO);
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

    Ptr<NrGnbRrc> rrc = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
    Ptr<NrMacSchedulerNs3> schedRt =
        DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(gnbDevs.Get(0), 0));

    static WindowStats s0;
    static WindowStats s1;
    static WindowStats s2;
    static WindowStats s3;

    Simulator::Schedule(appStart, &TakeSnapshot, monitor, schedRt, &s0);
    Simulator::Schedule(MilliSeconds(400), &TakeSnapshot, monitor, schedRt, &s1);
    Simulator::Schedule(MilliSeconds(400), &NrGnbRrc::SetEnableHarq, rrc, false);
    Simulator::Schedule(MilliSeconds(700), &TakeSnapshot, monitor, schedRt, &s2);
    Simulator::Schedule(MilliSeconds(700), &NrGnbRrc::SetEnableHarq, rrc, true);
    Simulator::Schedule(simTime - NanoSeconds(1), &TakeSnapshot, monitor, schedRt, &s3);

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
    // ---- Per-window results ----
    auto printWindow = [&](const std::string& label,
                           const WindowStats& start,
                           const WindowStats& end,
                           double durationS) {
        const uint64_t rxPkts = end.rxPackets - start.rxPackets;
        const uint64_t rxBytes = end.rxBytes - start.rxBytes;
        const uint64_t firstTx = end.harq.totalTbFirstTx - start.harq.totalTbFirstTx;
        const uint64_t retx = end.harq.totalHarqRetx - start.harq.totalHarqRetx;
        const uint64_t nack = end.harq.totalTbFirstTxNack - start.harq.totalTbFirstTxNack;

        const double thr = durationS > 0 ? (rxBytes * 8.0) / durationS / 1e6 : 0.0;
        const double avgRetx = firstTx ? static_cast<double>(retx) / firstTx : 0.0;
        const double blerBefore = firstTx ? static_cast<double>(nack) / firstTx : 0.0;

        std::cout << "| " << std::setw(24) << std::left << label << " | " << std::setw(8)
                  << std::right << rxPkts << " | " << std::setw(10) << std::fixed
                  << std::setprecision(3) << thr << " | " << std::setw(9) << std::setprecision(3)
                  << avgRetx << " | " << std::setw(10) << std::setprecision(3) << blerBefore
                  << " |\n";
    };

    std::cout << "\n===== Per-window results (HARQ toggled at runtime via RRC) =====\n";
    std::cout << "| Window                   | Rx pkts  | Thr (Mbps) | Avg retx  | BLER 1st tx |\n";
    std::cout << "|--------------------------|----------|------------|-----------|------------|\n";
    printWindow("0.2-0.4 s  HARQ on", s0, s1, 0.2);
    printWindow("0.4-0.7 s  HARQ off", s1, s2, 0.3);
    printWindow("0.7-1.0 s  HARQ on", s2, s3, 0.3);

    Simulator::Destroy();
    return 0;
}
