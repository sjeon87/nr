// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-metrics-helper.h"
#include "ns3/nr-radio-setup-helper.h"
#include "ns3/nr-traffic-helper.h"

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

  Ptr<NrRadioSetupHelper> radioSetup = CreateObject<NrRadioSetupHelper>();
  radioSetup->ApplyAntennaProfile(nrHelper, NrAntennaProfile{4, 8, 2, 4, false});
  NrInstalledDevices devices =
    radioSetup->InstallDevices(nrHelper,
                               gridScenario.GetBaseStations(),
                               gridScenario.GetUserTerminals(),
                               allBwps);

  Ptr<NrInternetHelper> net = CreateObject<NrInternetHelper>();
  net->SetEpcHelper(nrEpcHelper);
  net->SetBackhaulAttributes(DataRate("100Gb/s"), 2500, Seconds(0.0));
  NrInternetEndpoints ep = net->SetupFullInternet(gridScenario.GetUserTerminals(), devices.ue);

  radioSetup->AttachToClosest(nrHelper, devices.ue, devices.gnb);

  Ptr<NrTrafficHelper> traffic = CreateObject<NrTrafficHelper>();
  NrUdpFlowSpec dl;
  dl.direction = NrTrafficDirection::DOWNLINK;
  dl.port = 1234;
  dl.packetSize = 100;
  dl.interval = MilliSeconds(1);
  dl.start = appStartTime;
  dl.stop = simTime;
  traffic->InstallUdpFlow(dl,
                          ep.remoteHost,
                          gridScenario.GetUserTerminals(),
                          ep.ueIfaces,
                          nrHelper,
                          devices.ue);

  Ptr<NrMetricsHelper> metrics = CreateObject<NrMetricsHelper>();
  NodeContainer endpoints;
  endpoints.Add(ep.remoteHost);
  endpoints.Add(gridScenario.GetUserTerminals());
  Ptr<FlowMonitor> monitor = metrics->InstallFlowMonitor(endpoints);

  Simulator::Stop(simTime);
  Simulator::Run();

  monitor->CheckForLostPackets();
  auto stats = monitor->GetFlowStats();
  std::cout << "Flows observed: " << stats.size() << std::endl;

  Simulator::Destroy();
  return 0;
}