// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5 - gNB energy model
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy model

/**
 * @file gsoc-nr-energy-example.cc
 * @ingroup examples
 *
 * @brief Minimal end-to-end example of the NR energy framework via NrEnergyHelper.
 *
 * Builds a standard 5G-LENA downlink scenario (1 gNB, a few UEs, UDP traffic on
 * a single FR1 band matching the gNB model's TR 38.864 Set 1 defaults) and
 * installs the whole energy stack with NrEnergyHelper. The helper creates each
 * device's energy model, connects it to a caller-provided energy::EnergySource,
 * and attaches the PHY listeners that translate live scheduler / PHY events into
 * TR 38.864 (gNB) and TR 38.840 (UE) power-state changes; with EnableDrx() it
 * also installs a C-DRX model per UE.
 *
 * The flow mirrors the six steps a user scenario follows:
 *   1. build the NR scenario (unchanged);
 *   2. install energy sources with the standard ns-3 energy module;
 *   3. create NrEnergyHelper and configure only what differs from the defaults;
 *   4. InstallGnb() / InstallUe() - the whole energy integration;
 *   5. (optional) connect traces / read results however the user likes;
 *   6. Run(), then read the energy off the returned model containers.
 *
 * @section out What the example prints
 *
 * After the run it reports, from the model containers the helper returned:
 *   - gNB total energy [J] and average power [W] over the simulated time. The
 *     average power sits between the TR 38.864 Table 5.1-3 micro sleep value
 *     (P3, the floor of an idle but awake carrier) and the active DL value
 *     (P4), and moves with the offered load;
 *   - per-UE energy [J] and average power [W], plus the UE total. With
 *     --enableDrx=1 the UEs sleep through the idle tail, so this figure drops
 *     against --enableDrx=0 while the gNB figure barely moves;
 *   - network efficiency in Mbit/J: received application bits over gNB energy,
 *     the usual "bits per Joule" energy-efficiency metric.
 *
 * The same numbers are available live rather than as totals: every model
 * exposes an "InstantaneousPower" and a "TotalEnergyConsumption" trace source.
 *
 * @section cfg 3GPP configuration
 *
 * --refConfigSet selects a named TR 38.864 Table 5.1-1 reference
 * configuration. A named set is one coherent bundle - it fixes the reference
 * Tx power and the P1..P5 row of Table 5.1-3, and it requires the PHY
 * numerology to match the set's subcarrier spacing - so the example derives
 * the numerology and the gNB Tx power from it rather than letting them drift
 * apart. Custom applies no preset and leaves every reference parameter as
 * configured here.
 *
 * Usage:
 *   ./ns3 run "gsoc-nr-energy-example --ueNumPergNb=4 --simTime=400ms --enableDrx=1"
 *   ./ns3 run "gsoc-nr-energy-example --numBwp=2"
 *   ./ns3 run "gsoc-nr-energy-example --refConfigSet=Set3"
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

NS_LOG_COMPONENT_DEFINE("GsocNrEnergyExample");

int
main(int argc, char* argv[])
{
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 4;
    Time simTime = MilliSeconds(400);
    Time udpAppStartTime = MilliSeconds(100);
    double centralFrequency = 3.5e9; // FR1
    double bandwidth = 100e6;        // 100 MHz (TR 38.864 Set 1)
    uint8_t numBwp = 1;              // bandwidth parts in the single carrier
    std::string refConfigSet = "Set1";
    uint32_t udpPacketSize = 1252;
    uint32_t lambda = 10000; // packets/s per UE
    bool enableDrx = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("ueNumPergNb", "Number of UEs per gNB", ueNumPergNb);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numBwp", "Bandwidth parts in the component carrier", numBwp);
    cmd.AddValue("refConfigSet",
                 "TR 38.864 Table 5.1-1 reference configuration: Set1, Set2, Set3 or Custom",
                 refConfigSet);
    cmd.AddValue("lambda", "UDP packets per second per UE", lambda);
    cmd.AddValue("enableDrx", "Install a C-DRX model on each UE", enableDrx);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(numBwp < 1, "numBwp must be at least 1");

    // A named reference set is a bundle, not a single number: Table 5.1-1 fixes
    // the frequency range, the subcarrier spacing and the reference Tx power
    // together, and Table 5.1-3 keys P1..P5 off the same set. Deriving the
    // numerology and the Tx power from it here is what keeps the scenario and
    // the energy model describing the same base station.
    uint16_t numerology = 1;    // Set1: 30 kHz SCS
    double totalTxPower = 55;   // Set1: 55 dBm reference Tx power
    if (refConfigSet == "Set2") //  FR1 FDD, 20 MHz, 15 kHz, 49 dBm
    {
        numerology = 0;
        totalTxPower = 49;
        bandwidth = 20e6;
    }
    else if (refConfigSet == "Set3") // FR2 TDD, 100 MHz, 120 kHz, 33 dBm
    {
        numerology = 3;
        totalTxPower = 33;
        centralFrequency = 28e9;
    }
    else if (refConfigSet == "Custom") // no preset; the values below stand alone
    {
        totalTxPower = 43;
    }

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ---------------------------------------------------------------------
    // 1. Build the NR scenario (nothing energy-specific here).
    // ---------------------------------------------------------------------
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);
    gridScenario.SetHorizontalBsDistance(10.0);
    gridScenario.SetBsHeight(10.0);
    gridScenario.SetUtHeight(1.5);
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNbNum);
    gridScenario.SetUtNumber(ueNumPergNb * gNbNum);
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

    // One operation band, one component carrier, numBwp bandwidth parts inside
    // it. All of them report into the SAME energy model: bandwidth parts of one
    // carrier have their occupancy aggregated and the TR 38.864 formula applied
    // once, so the static baseline P3 is counted once and not once per part.
    // For several CARRIERS see gsoc-nr-energy-ca-example.
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    bandConf.m_numBwp = numBwp;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

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
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Every bandwidth part needs its numerology and power: each has its own PHY
    // and its own listener, and the listener reads the symbol period off the PHY
    // as it attaches.
    for (uint32_t bwp = 0; bwp < allBwps.size(); ++bwp)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(0), bwp)
            ->SetAttribute("Numerology", UintegerValue(numerology));
        NrHelper::GetGnbPhy(gnbNetDev.Get(0), bwp)
            ->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    // Internet / IP / attach.
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.0));
    InternetStackHelper internet;
    internet.Install(ueContainer);
    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // Downlink UDP traffic.
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    UdpServerHelper dlPacketSink(dlPort);
    serverApps.Add(dlPacketSink.Install(ueContainer));

    UdpClientHelper dlClient;
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda)));

    NrQosFlow dlFlow(NrQosFlow::NGBR_LOW_LAT_EMBB);
    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    {
        Address ueAddress = ueIpIface.GetAddress(i);
        dlClient.SetAttribute(
            "Remote",
            AddressValue(addressUtils::ConvertToSocketAddress(ueAddress, dlPort)));
        clientApps.Add(dlClient.Install(remoteHost));
        Ptr<NrQosRule> rule = Create<NrQosRule>();
        NrQosRule::PacketFilter pf;
        pf.localPortStart = dlPort;
        pf.localPortEnd = dlPort;
        rule->Add(pf);
        nrHelper->ActivateDedicatedQosFlow(ueNetDev.Get(i), dlFlow, rule);
    }
    // Send traffic only in the first half of the active window and leave an
    // idle tail: with DRX on, the UEs sleep through the tail (deep/light sleep);
    // with DRX off they keep monitoring PDCCH. This is what makes the DRX knob
    // visible in the reported UE energy. Under continuous back-to-back load the
    // inactivity timer never lapses, so DRX on and off would be nearly equal.
    Time trafficStopTime = udpAppStartTime + (simTime - udpAppStartTime) / 2;
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(trafficStopTime);

    // ---------------------------------------------------------------------
    // 2. Install energy sources with the standard ns-3 energy module: one
    //    BasicEnergySource per node, index-aligned with the device containers.
    // ---------------------------------------------------------------------
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1e9)); // large reservoir
    energy::EnergySourceContainer gnbSources = srcHelper.Install(gnbContainer);
    energy::EnergySourceContainer ueSources = srcHelper.Install(ueContainer);

    // ---------------------------------------------------------------------
    // 3. Create the energy helper and override only what we care about; every
    //    other knob keeps its TR 38.864 / TR 38.840 default.
    // ---------------------------------------------------------------------
    NrEnergyHelper energyHelper;

    // A 3GPP-compliant gNB configuration is these four together. BsCategory
    // picks the hardware class (Cat 1 macro or Cat 2 small cell) and
    // RefConfigSet the Table 5.1-1 bundle; between them they select the
    // Table 5.1-3 row that gives P1..P5. AntennaRatio_A is the share of the
    // dynamic power that belongs to the antenna (Section 5.1 baseline 0.4;
    // 0.1 and 0.7 are the values the spec offers as alternatives), and EtaMode
    // chooses between a constant PA efficiency and the two-valued one that
    // drops to 0.76 below sf*sp = 0.5.
    energyHelper.SetGnbEnergyModelAttribute("BsCategory", StringValue("BsCat1"));
    energyHelper.SetGnbEnergyModelAttribute("RefConfigSet", StringValue(refConfigSet));
    energyHelper.SetGnbEnergyModelAttribute("AntennaRatio_A", DoubleValue(0.4));
    energyHelper.SetGnbEnergyModelAttribute("EtaMode", StringValue("Dual"));

    // Only meaningful with RefConfigSet=Custom: a named set fixes the reference
    // Tx power itself, and overriding it would break the bundle.
    if (refConfigSet == "Custom")
    {
        energyHelper.SetGnbEnergyModelAttribute("ReferenceTxPowerDbm", DoubleValue(totalTxPower));
    }
    energyHelper.EnableDrx(enableDrx);

    // ---------------------------------------------------------------------
    // 4. Install - one call per side. This is the whole energy integration.
    // ---------------------------------------------------------------------
    energy::DeviceEnergyModelContainer gnbModels = energyHelper.InstallGnb(gnbNetDev, gnbSources);
    energy::DeviceEnergyModelContainer ueModels = energyHelper.InstallUe(ueNetDev, ueSources);

    // ---------------------------------------------------------------------
    // 5. (optional) connect traces here, e.g. the model's TotalEnergyConsumption
    //    trace source, if a time series is wanted.
    // 6. Run and read the numbers off the returned containers afterwards.
    // ---------------------------------------------------------------------
    Simulator::Stop(simTime);
    Simulator::Run();

    double simSeconds = simTime.GetSeconds();
    uint64_t rxPackets = 0;
    for (uint32_t i = 0; i < serverApps.GetN(); ++i)
    {
        rxPackets += DynamicCast<UdpServer>(serverApps.Get(i))->GetReceived();
    }
    double rxBits = static_cast<double>(rxPackets) * udpPacketSize * 8.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n================ NR Energy Example Results ================\n";
    std::cout << "Sim time: " << simSeconds << " s, UEs: " << ueContainer.GetN()
              << ", DRX: " << (enableDrx ? "on" : "off") << "\n";
    std::cout << "TR 38.864 " << refConfigSet << " (BsCat1), " << +numBwp
              << " BWP in 1 CC, numerology " << numerology << ", Tx " << totalTxPower << " dBm\n\n";

    double gnbEnergy = gnbModels.Get(0)->GetTotalEnergyConsumption();
    std::cout << "gNB:\n";
    std::cout << "  Total energy : " << gnbEnergy << " J\n";
    std::cout << "  Average power: " << gnbEnergy / simSeconds << " W\n\n";

    double ueEnergyTotal = 0.0;
    for (uint32_t i = 0; i < ueModels.GetN(); ++i)
    {
        double e = ueModels.Get(i)->GetTotalEnergyConsumption();
        ueEnergyTotal += e;
        std::cout << "  UE" << i << " energy: " << e << " J  (avg " << e / simSeconds << " W)\n";
    }
    std::cout << "  UE energy total: " << ueEnergyTotal << " J\n\n";

    if (gnbEnergy > 0.0)
    {
        std::cout << "Network efficiency: " << (rxBits / gnbEnergy) / 1e6 << " Mbit/J\n";
    }
    std::cout << "===========================================================\n";

    Simulator::Destroy();
    return 0;
}
