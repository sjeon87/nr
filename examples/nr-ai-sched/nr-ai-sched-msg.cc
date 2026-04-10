// Copyright (c) 2026 University of Moratuwa
// Author: Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
/**
 * @ingroup examples
 * @file nr-ai-sched-msg.cc
 * @brief NR AI scheduler example using ns-3-ai msg-interface.
 *
 * This example demonstrates how to use the ns-3-ai msg-interface for
 * AI-driven MAC scheduling in 5G NR.  It replaces the ZMQ/protobuf
 * IPC path from GSoC 2024 with Boost.Interprocess shared memory,
 * achieving microsecond level per TTI data exchange.
 *
 * Scenario:
 * - 1 gNB, N UEs arranged in a GridScenario
 * - Even UEs: 1 flow (eMBB, NGBR_LOW_LAT_EMBB, 5QI=9)
 * - Odd UEs:  2 flows (eMBB + URLLC via GBR_CONV_VOICE, 5QI=1)
 * - The AI scheduler weights are set via shared memory by a Python
 *   PPO agent (nr_ai_sched_ppo.py), run in a separate process.
 *
 * Running:
 * @code
 *   # Terminal 1: Start the C++ simulation
 *   ./ns3 run "nr-ai-sched-msg --ueNum=4 --simTime=2000ms"
 *
 *   # Terminal 2: Start the Python PPO agent
 *   python3 contrib/nr/examples/nr-ai-sched/nr_ai_sched_ppo.py
 * @endcode
 *
 * @note The Python agent must be started AFTER the C++ simulation,
 *       because C++ creates the shared memory segment.
 *
 * @see NrMacSchedulerAiMsgEnv
 * @see NrSchedEnvMsg, NrSchedActMsg
 */
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
// Include the shared-memory bridge (conditionally compiled)
#ifdef HAVE_NS3_AI
#include "ns3/nr-mac-scheduler-ai-msg-env.h"
#endif
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("NrAiSchedMsg");
int main(int argc, char *argv[]) {

  // Commandline parameters

  uint16_t ueNum = 4;
  bool logging = false;
  Time simTime = MilliSeconds(2000);
  Time udpAppStartTime = MilliSeconds(400);
  uint16_t numerology = 0;
  double centralFrequency = 4e9;
  double bandwidth = 10e6;
  double totalTxPower = 43;
  uint8_t enableOfdma = 0;
  std::string schedulerType = "Ai";
  uint8_t enableQoSLcScheduler = 0;
  uint8_t priorityTrafficScenario = 0;
  uint16_t mcsTable = 2;
  std::string simTag = "default";
  std::string outputDir = "./";
  CommandLine cmd;
  cmd.AddValue("ueNum", "Number of UEs per gNB", ueNum);
  cmd.AddValue("logging", "Enable logging", logging);
  cmd.AddValue("priorityTrafficScenario", "0: saturation, 1: medium-load",
               priorityTrafficScenario);
  cmd.AddValue("simTime", "Simulation time", simTime);
  cmd.AddValue("numerology", "NR numerology (0-4)", numerology);
  cmd.AddValue("centralFrequency", "Central frequency in Hz", centralFrequency);
  cmd.AddValue("bandwidth", "System bandwidth in Hz", bandwidth);
  cmd.AddValue("totalTxPower", "Total Tx power in dBm", totalTxPower);
  cmd.AddValue("simTag", "Tag appended to output filenames", simTag);
  cmd.AddValue("outputDir", "Directory for output files", outputDir);
  cmd.AddValue("enableOfdma", "1 = OFDMA, 0 = TDMA (default)", enableOfdma);
  cmd.AddValue("schedulerType", "Scheduler algorithm: PF, RR, Qos, or Ai",
               schedulerType);
  cmd.AddValue("enableLcLevelQos", "Enable QoS-aware LC scheduler",
               enableQoSLcScheduler);
  cmd.Parse(argc, argv);
  // Validate that we have ns-3-ai when the user requests AI
#ifndef HAVE_NS3_AI
  NS_ABORT_MSG_IF(schedulerType == "Ai",
                  "ns-3-ai module is not available.  Build with contrib/ai "
                  "to use the AI scheduler via msg-interface.");
#endif
  if (logging) {
    LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME |
                                   LOG_PREFIX_NODE | LOG_LEVEL_INFO);
    LogComponentEnable("NrMacSchedulerNs3", logLevel);
    LogComponentEnable("NrMacSchedulerTdma", logLevel);
#ifdef HAVE_NS3_AI
    LogComponentEnable("NrMacSchedulerAiMsgEnv", logLevel);
#endif
  }
  Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

  // Topology: GridScenario with 1 gNB
  int64_t randomStream = 1;
  GridScenarioHelper gridScenario;
  gridScenario.SetRows(1);
  gridScenario.SetColumns(1);
  gridScenario.SetHorizontalBsDistance(5.0);
  gridScenario.SetVerticalBsDistance(5.0);
  gridScenario.SetBsHeight(1.5);
  gridScenario.SetUtHeight(1.5);
  gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
  gridScenario.SetBsNumber(1);
  gridScenario.SetUtNumber(ueNum);
  gridScenario.SetScenarioHeight(3);
  gridScenario.SetScenarioLength(3);
  randomStream += gridScenario.AssignStreams(randomStream);
  gridScenario.CreateScenario();
  // Traffic parameters
  uint32_t udpPacketSize1;
  uint32_t udpPacketSize2;
  uint32_t lambda1 = 1000;
  uint32_t lambda2 = 1000;
  if (priorityTrafficScenario == 0) {
    udpPacketSize1 = 3000; // eMBB saturation
    udpPacketSize2 = 3000; // URLLC saturation
  } else if (priorityTrafficScenario == 1) {
    udpPacketSize1 = 3000;
    udpPacketSize2 = 1252; // Medium-load URLLC
  } else {
    NS_ABORT_MSG("priorityTrafficScenario must be 0 or 1");
  }
  // Separate UEs by traffic type
  // Even indexed UEs: 1 flow (eMBB only)
  // Odd indexed UEs:  2 flows (eMBB + URLLC)
  NodeContainer ue1flowContainer;  // eMBB only UEs
  NodeContainer ue2flowsContainer; // eMBB + URLLC UEs
  for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j) {
    Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);
    if (j % 2 == 0) {
      ue1flowContainer.Add(ue);
    } else {
      ue2flowsContainer.Add(ue);
    }
  }
  if (priorityTrafficScenario == 1) {
    lambda1 = 1000 / ue1flowContainer.GetN();
    lambda2 = 1000 / ue2flowsContainer.GetN();
  }
  // NR stack setup
  Ptr<NrPointToPointEpcHelper> epcHelper =
      CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> idealBeamformingHelper =
      CreateObject<IdealBeamformingHelper>();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  nrHelper->SetBeamformingHelper(idealBeamformingHelper);
  nrHelper->SetEpcHelper(epcHelper);
  epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
  // Set the scheduler type
  std::stringstream scheduler;
  std::string subType = !enableOfdma ? "Tdma" : "Ofdma";
  scheduler << "ns3::NrMacScheduler" << subType << schedulerType;
  std::cout << "Scheduler: " << scheduler.str() << std::endl;
  nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler.str()));
  // Connect to the ns-3-ai msg-interface bridge
#ifdef HAVE_NS3_AI
  // Total expected flows: 1 flow UEs contribute 1 each,
  // 2 flow UEs contribute 2 each
  uint32_t totalFlows = ue1flowContainer.GetN() + ue2flowsContainer.GetN() * 2;
  Ptr<NrMacSchedulerAiMsgEnv> msgEnv =
      CreateObject<NrMacSchedulerAiMsgEnv>(totalFlows);
  if (schedulerType == "Ai") {
    nrHelper->SetSchedulerAttribute(
        "NotifyCbDl",
        CallbackValue(MakeCallback(
            &NrMacSchedulerAiMsgEnv::NotifyCurrentIteration, msgEnv)));
    nrHelper->SetSchedulerAttribute("ActiveDlAi", BooleanValue(true));
    std::cout << "AI scheduler (msg-interface) is enabled" << std::endl;
  }
#endif
  // QoS LC scheduler
  if (enableQoSLcScheduler) {
    nrHelper->SetSchedulerAttribute(
        "SchedLcAlgorithmType", TypeIdValue(NrMacSchedulerLcQos::GetTypeId()));
    std::cout << "QoS LC scheduler is enabled" << std::endl;
  }
  // Error model
  std::string errorModel = "ns3::NrEesmIrT" + std::to_string(mcsTable);
  nrHelper->SetDlErrorModel(errorModel);
  nrHelper->SetUlErrorModel(errorModel);
  nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
  nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
  // Beamforming
  idealBeamformingHelper->SetAttribute(
      "BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
  // Antennas
  nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
  nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
  nrHelper->SetUeAntennaAttribute(
      "AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
  nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
  nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
  nrHelper->SetGnbAntennaAttribute(
      "AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

  // Spectrum configuration: 1 CC, 1 BWP
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  OperationBandInfo band;
  Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
  channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
  auto bandMask = NrChannelHelper::INIT_PROPAGATION;
  channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
  Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                     TimeValue(MilliSeconds(0)));
  channelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                   TimeValue(MilliSeconds(0)));
  CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth,
                                                 1);
  bandConf.m_numBwp = 1;
  band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  channelHelper->AssignChannelsToBands({band}, bandMask);
  allBwps = CcBwpCreator::GetAllBwps({band});
  double x = pow(10, totalTxPower / 10);
  Packet::EnableChecking();
  Packet::EnablePrinting();
  // BWP routing: eMBB + URLLC on same BWP 0
  uint32_t bwpId = 0;
  nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",
                                               UintegerValue(bwpId));
  nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE",
                                               UintegerValue(bwpId));
  nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",
                                              UintegerValue(bwpId));
  nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE",
                                              UintegerValue(bwpId));
  // Install devices
  NetDeviceContainer enbNetDev =
      nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
  NetDeviceContainer ue1flowNetDev =
      nrHelper->InstallUeDevice(ue1flowContainer, allBwps);
  NetDeviceContainer ue2flowsNetDev =
      nrHelper->InstallUeDevice(ue2flowsContainer, allBwps);
  NetDeviceContainer ueNetDevs(ue1flowNetDev);
  ueNetDevs.Add(ue2flowsNetDev);
  randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
  randomStream += nrHelper->AssignStreams(ueNetDevs, randomStream);
  NrHelper::GetGnbPhy(enbNetDev.Get(0), 0)
      ->SetAttribute("Numerology", UintegerValue(numerology));
  NrHelper::GetGnbPhy(enbNetDev.Get(0), 0)
      ->SetAttribute("TxPower", DoubleValue(10 * log10(x)));
  // Internet stack
  Ptr<Node> pgw = epcHelper->GetPgwNode();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  InternetStackHelper internet;
  internet.Install(remoteHostContainer);
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                             Ipv4Mask("255.0.0.0"), 1);
  internet.Install(gridScenario.GetUserTerminals());
  Ipv4InterfaceContainer ue1FlowIpIface =
      epcHelper->AssignUeIpv4Address(NetDeviceContainer(ue1flowNetDev));
  Ipv4InterfaceContainer ue2FlowsIpIface =
      epcHelper->AssignUeIpv4Address(NetDeviceContainer(ue2flowsNetDev));
  nrHelper->AttachToClosestGnb(ueNetDevs, enbNetDev);
  // Traffic: eMBB (NGBR_LOW_LAT_EMBB) + URLLC (GBR_CONV_VOICE)
  // Using GBR_CONV_VOICE (5QI = 1) for URLLC instead of
  // DGBR_INTER_SERV_87 from the GSoC 2024 example, because 5QI=1
  // has a strict 100ms PDB which better exercises the Lyapunov
  // reward function's delay penalty term.
  uint16_t dlPortEmbb = 1234;
  uint16_t dlPortEmbb2 = 1235;
  uint16_t dlPortUrllc = 1236;
  ApplicationContainer serverApps;
  UdpServerHelper dlPacketSinkEmbb(dlPortEmbb);
  UdpServerHelper dlPacketSinkEmbb2(dlPortEmbb2);
  UdpServerHelper dlPacketSinkUrllc(dlPortUrllc);
  serverApps.Add(dlPacketSinkEmbb.Install(ue1flowContainer));
  serverApps.Add(dlPacketSinkEmbb2.Install(ue2flowsContainer));
  serverApps.Add(dlPacketSinkUrllc.Install(ue2flowsContainer));
  // eMBB traffic for 1 flow UEs
  UdpClientHelper dlClientEmbb;
  dlClientEmbb.SetAttribute("RemotePort", UintegerValue(dlPortEmbb));
  dlClientEmbb.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  dlClientEmbb.SetAttribute("PacketSize", UintegerValue(udpPacketSize1));
  dlClientEmbb.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda1)));
  NrQosFlow embbFlow(NrQosFlow::NGBR_LOW_LAT_EMBB);
  Ptr<NrQosRule> embbRule = Create<NrQosRule>();
  NrQosRule::PacketFilter embbFilter;
  embbFilter.localPortStart = dlPortEmbb;
  embbFilter.localPortEnd = dlPortEmbb;
  embbRule->Add(embbFilter);
  // eMBB traffic for 2 flow UEs (Non GBR part)
  UdpClientHelper dlClientEmbb2;
  dlClientEmbb2.SetAttribute("RemotePort", UintegerValue(dlPortEmbb2));
  dlClientEmbb2.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  dlClientEmbb2.SetAttribute("PacketSize", UintegerValue(udpPacketSize1));
  dlClientEmbb2.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda1)));
  NrQosFlow embb2Flow(NrQosFlow::NGBR_LOW_LAT_EMBB);
  Ptr<NrQosRule> embb2Rule = Create<NrQosRule>();
  NrQosRule::PacketFilter embb2Filter;
  embb2Filter.localPortStart = dlPortEmbb2;
  embb2Filter.localPortEnd = dlPortEmbb2;
  embb2Rule->Add(embb2Filter);
  // URLLC traffic for 2 flow UEs (GBR_CONV_VOICE, 5QI=1)
  UdpClientHelper dlClientUrllc;
  dlClientUrllc.SetAttribute("RemotePort", UintegerValue(dlPortUrllc));
  dlClientUrllc.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  dlClientUrllc.SetAttribute("PacketSize", UintegerValue(udpPacketSize2));
  dlClientUrllc.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda2)));
  NrGbrQosInformation qosUrllc;
  qosUrllc.gbrDl = 5e6; // 5 Mbps guaranteed DL
  NrQosFlow urllcFlow(NrQosFlow::GBR_CONV_VOICE, qosUrllc);
  Ptr<NrQosRule> urllcRule = Create<NrQosRule>();
  NrQosRule::PacketFilter urllcFilter;
  urllcFilter.localPortStart = dlPortUrllc;
  urllcFilter.localPortEnd = dlPortUrllc;
  urllcRule->Add(urllcFilter);
  // Install applications and activate QoS flows
  ApplicationContainer clientApps;
  std::map<std::pair<Address, uint16_t>, std::string> flowMap;
  // 1 flow UEs (eMBB only)
  for (uint32_t i = 0; i < ue1flowContainer.GetN(); ++i) {
    Ptr<NetDevice> ueDevice = ue1flowNetDev.Get(i);
    Address ueAddress = ue1FlowIpIface.GetAddress(i);
    dlClientEmbb.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    clientApps.Add(dlClientEmbb.Install(remoteHost));
    nrHelper->ActivateDedicatedQosFlow(ueDevice, embbFlow, embbRule);
    std::stringstream flowType;
    flowType << "UE " << ueDevice->GetNode()->GetId() << " eMBB";
    flowMap.emplace(std::make_pair(ueAddress, dlPortEmbb), flowType.str());
  }
  // 2 flow UEs (eMBB + URLLC)
  for (uint32_t i = 0; i < ue2flowsContainer.GetN(); ++i) {
    Ptr<NetDevice> ueDevice = ue2flowsNetDev.Get(i);
    Address ueAddress = ue2FlowsIpIface.GetAddress(i);
    // eMBB flow
    dlClientEmbb2.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    clientApps.Add(dlClientEmbb2.Install(remoteHost));
    nrHelper->ActivateDedicatedQosFlow(ueDevice, embb2Flow, embb2Rule);
    std::stringstream flowType1;
    flowType1 << "UE " << ueDevice->GetNode()->GetId() << " eMBB";
    flowMap.emplace(std::make_pair(ueAddress, dlPortEmbb2), flowType1.str());
    // URLLC flow
    dlClientUrllc.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    clientApps.Add(dlClientUrllc.Install(remoteHost));
    nrHelper->ActivateDedicatedQosFlow(ueDevice, urllcFlow, urllcRule);
    std::stringstream flowType2;
    flowType2 << "UE " << ueDevice->GetNode()->GetId() << " URLLC";
    flowMap.emplace(std::make_pair(ueAddress, dlPortUrllc), flowType2.str());
  }
  // Start/stop applications
  serverApps.Start(udpAppStartTime);
  clientApps.Start(udpAppStartTime);
  serverApps.Stop(simTime);
  clientApps.Stop(simTime);
  // FlowMonitor
  FlowMonitorHelper flowmonHelper;
  NodeContainer endpointNodes;
  endpointNodes.Add(remoteHost);
  endpointNodes.Add(gridScenario.GetUserTerminals());
  Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
  monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));
  // Run
  Simulator::Stop(simTime);
  Simulator::Run();
  // Print per-flow statistics
  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  double averageFlowThroughput = 0.0;
  double averageFlowDelay = 0.0;
  std::ofstream outFile;
  std::string filename = outputDir + "/" + simTag;
  outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!outFile.is_open()) {
    std::cerr << "Can't open file " << filename << std::endl;
    return 1;
  }
  outFile.setf(std::ios_base::fixed);
  double flowDuration = (simTime - udpAppStartTime).GetSeconds();
  for (const auto &[flowId, flowStats] : stats) {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
    std::stringstream protoStream;
    if (t.protocol == 17) {
      protoStream << "UDP";
    } else {
      protoStream << static_cast<uint16_t>(t.protocol);
    }
    auto flowKey = std::make_pair(t.destinationAddress, t.destinationPort);
    std::string flowLabel = "Unknown";
    if (flowMap.count(flowKey)) {
      flowLabel = flowMap.at(flowKey);
    }
    outFile << "Flow " << flowId << " (" << t.sourceAddress << ":"
            << t.sourcePort << " -> " << t.destinationAddress << ":"
            << t.destinationPort << ") proto " << protoStream.str() << "\n";
    outFile << "  Flow Type: " << flowLabel << "\n";
    outFile << "  Tx Packets: " << flowStats.txPackets << "\n";
    outFile << "  Tx Bytes:   " << flowStats.txBytes << "\n";
    outFile << "  TxOffered:  " << flowStats.txBytes * 8.0 / flowDuration / 1e6
            << " Mbps\n";
    outFile << "  Rx Bytes:   " << flowStats.rxBytes << "\n";
    if (flowStats.rxPackets > 0) {
      double tput = flowStats.rxBytes * 8.0 / flowDuration / 1e6;
      double delay =
          1000 * flowStats.delaySum.GetSeconds() / flowStats.rxPackets;
      double jitter =
          1000 * flowStats.jitterSum.GetSeconds() / flowStats.rxPackets;
      averageFlowThroughput += tput;
      averageFlowDelay += delay;
      outFile << "  Throughput: " << tput << " Mbps\n";
      outFile << "  Mean delay:  " << delay << " ms\n";
      outFile << "  Mean jitter: " << jitter << " ms\n";
    } else {
      outFile << "  Throughput:  0 Mbps\n";
      outFile << "  Mean delay:  0 ms\n";
      outFile << "  Mean jitter: 0 ms\n";
    }
    outFile << "  Rx Packets: " << flowStats.rxPackets << "\n";
  }
  outFile << "\n  Mean flow throughput: "
          << averageFlowThroughput / stats.size() << "\n";
  outFile << "  Mean flow delay: " << averageFlowDelay / stats.size() << "\n";
  outFile.close();
  // Print to stdout
  std::ifstream f(filename.c_str());
  if (f.is_open()) {
    std::cout << f.rdbuf();
  }
  // Cleanup
#ifdef HAVE_NS3_AI
  if (schedulerType == "Ai" && msgEnv) {
    msgEnv->Dispose();
  }
#endif
  Simulator::Destroy();
  return 0;
}