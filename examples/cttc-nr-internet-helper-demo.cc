// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrInternetHelperDemo");

int
main(int argc, char* argv[])
{
  uint16_t gNbNum = 1;
  uint16_t ueNumPergNb = 2;
  Time simTime = Seconds(1.0);
  Time appStartTime = Seconds(0.4);

  CommandLine cmd(__FILE__);
  cmd.AddValue("gNbNum", "Number of gNBs", gNbNum);
  cmd.AddValue("ueNumPergNb", "UEs per gNB", ueNumPergNb);
  cmd.AddValue("simTime", "Simulation time", simTime);
  cmd.Parse(argc, argv);

  GridScenarioHelper gridScenario;
  gridScenario.SetRows(1);
  gridScenario.SetColumns(gNbNum);
  gridScenario.SetHorizontalBsDistance(10.0);
  gridScenario.SetVerticalBsDistance(10.0);
  gridScenario.SetBsHeight(10.0);
  gridScenario.SetUtHeight(1.5);
  gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
  gridScenario.SetBsNumber(gNbNum);
  gridScenario.SetUtNumber(ueNumPergNb * gNbNum);
  gridScenario.SetScenarioLength(50.0);
  gridScenario.SetScenarioHeight(50.0);
  gridScenario.CreateScenario();

  Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  nrHelper->SetEpcHelper(nrEpcHelper);
  nrHelper->SetBeamformingHelper(idealBeamformingHelper);

  CcBwpCreator ccBwpCreator;
  CcBwpCreator::SimpleOperationBandConf bandConf(28e9, 100e6, 1);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

  Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
  channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
  channelHelper->AssignChannelsToBands({band});
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

  nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
  nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
  nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
  nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));

  NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
  NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(gridScenario.GetUserTerminals(), allBwps);

  // Internet helper usage starts here.
  Ptr<NrInternetHelper> nrInternetHelper = CreateObject<NrInternetHelper>();
  nrInternetHelper->SetEpcHelper(nrEpcHelper);
  nrInternetHelper->SetBackhaulAttributes(DataRate("100Gb/s"), 2500, Seconds(0.0));
  nrInternetHelper->SetupRemoteHostIpv4();

  // Install UE stack and assign EPC-managed UE addresses.
  InternetStackHelper internet;
  internet.Install(gridScenario.GetUserTerminals());
  Ipv4InterfaceContainer ueIpIfaces = nrInternetHelper->AssignUeIpv4(ueNetDev);
  nrInternetHelper->SetupUeIpv4DefaultRoutes(gridScenario.GetUserTerminals());

  // Attach after IPv4 is configured on UEs, otherwise default bearer activation asserts.
  nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

  uint16_t dlPort = 1234;
  UdpServerHelper dlPacketSink(dlPort);
  ApplicationContainer serverApps = dlPacketSink.Install(gridScenario.GetUserTerminals());

  UdpClientHelper dlClient;
  dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
  dlClient.SetAttribute("PacketSize", UintegerValue(100));
  dlClient.SetAttribute("Interval", TimeValue(Seconds(0.001)));

  ApplicationContainer clientApps;
  for (uint32_t i = 0; i < ueIpIfaces.GetN(); ++i)
  {
    // Configure one downlink UDP source per UE from the remote host.
    dlClient.SetAttribute("RemoteAddress", AddressValue(ueIpIfaces.GetAddress(i)));
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    clientApps.Add(dlClient.Install(nrInternetHelper->GetRemoteHost()));
  }

  serverApps.Start(appStartTime);
  clientApps.Start(appStartTime);
  serverApps.Stop(simTime);
  clientApps.Stop(simTime);

  // Observe end-to-end traffic statistics on remote host + UE endpoints.
  FlowMonitorHelper flowmonHelper;
  NodeContainer endpointNodes;
  endpointNodes.Add(nrInternetHelper->GetRemoteHost());
  endpointNodes.Add(gridScenario.GetUserTerminals());
  Ptr<FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);

  Simulator::Stop(simTime);
  Simulator::Run();

  monitor->CheckForLostPackets();
  auto stats = monitor->GetFlowStats();
  std::cout << "Flows observed: " << stats.size() << std::endl;

  Simulator::Destroy();
  return 0;
}