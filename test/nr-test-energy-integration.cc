// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5 - Energy consumption model for BS
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/test.h"

#include <algorithm>
#include <bit>
#include <set>

using namespace ns3;

/**
 * @file nr-test-energy-integration.cc
 * @ingroup test
 *
 * @brief Integration tests: the NR device energy models and their attached
 * ns3::energy::EnergySource must account the same energy.
 *
 * These are deliberately not unit tests. They run a real NR scenario (EPC, a
 * real scheduler, real DL/UL traffic), drive the energy models only through the
 * public PHY listeners, and attach a real BasicEnergySource. The models are
 * therefore exercised exactly the way a user scenario exercises them, and no
 * private listener callback is invoked directly.
 *
 * The property under test is the one that makes the ns-3 energy framework
 * integration meaningful: a user who attaches an EnergySource may reasonably
 * read its remaining energy, so
 *
 *     analytical energy == model->GetTotalEnergyJ() == initial - remaining
 *
 * must hold. An EnergySource can only ever integrate the current reported by
 * DoGetCurrentA(), so this passes only if the model exposes one canonical power
 * and updates the source over the same intervals it charges internally.
 */
namespace ns3
{

/**
 * @brief Builds the shared NR scenario and the energy stack for both test cases.
 *
 * One gNB, one UE, EPC and a remote host, with DL UDP traffic that starts and
 * stops inside the simulation so the run contains busy slots, control-only
 * slots and fully idle slots. That spread is what makes the reconciliation
 * non-trivial: every slot has a different energy, so an off-by-one-slot or a
 * lump-sum charge shows up as a mismatch.
 */
class NrEnergyIntegrationTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param name Human readable test case name.
     */
    NrEnergyIntegrationTestCase(std::string name)
        : TestCase(name)
    {
    }

  protected:
    /**
     * @brief Build the scenario, wire the energy stack, and run it.
     *
     * Leaves the simulator stopped at m_simTime with both sources refreshed, so
     * the derived test can compare totals. Must be followed by Simulator::Destroy.
     *
     * @param enableDrx Install an NrUeDrxModel, so the UE actually sleeps and the
     *                  TR 38.840 Table 19 transition transient is exercised.
     */
    void RunScenario(bool enableDrx);

    /// Energy each source starts with; large enough that depletion never fires.
    static constexpr double INITIAL_ENERGY_J = 1e7;
    /// Supply voltage of the sources; a non-unity value catches W/A confusion.
    static constexpr double SUPPLY_VOLTAGE_V = 12.0;

    Time m_simTime{MilliSeconds(400)}; //!< Total simulated time

    Ptr<NrGnbEnergyModel> m_gnbModel;           //!< gNB model under test
    Ptr<NrUeEnergyModel> m_ueModel;             //!< UE model under test
    Ptr<energy::BasicEnergySource> m_gnbSource; //!< Real source on the gNB node
    Ptr<energy::BasicEnergySource> m_ueSource;  //!< Real source on the UE node
    Ptr<NrGnbPhy> m_gnbPhy;                     //!< gNB PHY driving the listener

    // ----- Independent analytical reconstruction, gNB -----

    /**
     * @brief Second subscriber to the PHY "SlotEnergyStats" trace.
     *
     * Recomputes the slot energy from the reported allocation using only the
     * public TR 38.864 formula accessors, independently of anything the model
     * accumulates internally. Connected after the listener, so the sp the
     * listener refreshed for this slot is the one in effect.
     *
     * @param sfnSf       Frame/slot number.
     * @param availableRb RBs available in the BWP.
     * @param dlDataSym   DL data symbols.
     * @param dlDataReg   DL data REGs.
     * @param ulDataSym   UL data symbols.
     * @param dlCtrlSym   DL control symbols.
     * @param dlCtrlReg   DL control REGs.
     * @param ulCtrlSym   UL control symbols.
     * @param bwpId       Bandwidth part id.
     * @param cellId      Cell id.
     */
    void AnalyticalSlot(const SfnSf& sfnSf,
                        uint32_t availableRb,
                        uint16_t dlDataMask,
                        uint32_t dlDataReg,
                        uint16_t ulMask,
                        uint16_t dlCtrlMask,
                        uint32_t dlCtrlReg,
                        uint16_t bwpId,
                        uint16_t cellId);

    double m_analyticalJ{0.0};           //!< Running analytical gNB energy [J]
    double m_pendingPowerW{0.0};         //!< Slot power installed for the interval in progress
    Time m_lastSlotTime{Seconds(0)};     //!< Start of that interval
    std::set<uint64_t> m_slotEnergyKeys; //!< Distinct slot energies seen (quantized)

    // ----- Independent reconstruction, UE -----

    /**
     * @brief Integrate the UE "InstantaneousPower" trace.
     *
     * The trace is piecewise constant and fires on every power change, including
     * the transition transient, so the Riemann sum below is exact rather than an
     * approximation. It reads only the public trace, never m_totalEnergyJ.
     *
     * @param oldPowerW Previous power [W].
     * @param newPowerW New power [W].
     */
    void UePowerChanged(double oldPowerW, double newPowerW);

    /// Set once the totals have been sampled; the reconstructions stop accruing.
    /// The simulator runs slightly past m_simTime so that the sampling event is
    /// guaranteed to execute, and without this flag the trace-driven sums would
    /// keep growing past the instant the models were read.
    bool m_sampled{false};

    double m_gnbFinalModelJ{0.0}; //!< gNB GetTotalEnergyJ() sampled at m_simTime
    double m_gnbFinalDrainJ{0.0}; //!< gNB source drain sampled at m_simTime
    double m_ueFinalModelJ{0.0};  //!< UE GetTotalEnergyJ() sampled at m_simTime
    double m_ueFinalDrainJ{0.0};  //!< UE source drain sampled at m_simTime

    /// Relative tolerance. Not 1e-12: BasicEnergySource accumulates its remaining
    /// energy by repeated subtraction over thousands of updates, so its float
    /// noise floor sits near 1e-8 relative. Still orders of magnitude tighter
    /// than any accounting bug this test is meant to catch.
    static constexpr double REL_TOL = 1e-6;

    double m_ueTraceJ{0.0};        //!< Running UE energy from the trace [J]
    double m_uePowerW{0.0};        //!< Power in effect since m_ueLastTime
    Time m_ueLastTime{Seconds(0)}; //!< Start of the current UE power interval
};

void
NrEnergyIntegrationTestCase::AnalyticalSlot(const SfnSf&,
                                            uint32_t availableRb,
                                            uint16_t dlDataMask,
                                            uint32_t dlDataReg,
                                            uint16_t ulMask,
                                            uint16_t dlCtrlMask,
                                            uint32_t dlCtrlReg,
                                            uint16_t,
                                            uint16_t)
{
    // The trace reports symbol POSITIONS; the analytical leg only needs counts,
    // which are their popcount.
    const uint32_t dlDataSym = std::popcount(dlDataMask);
    const uint32_t dlCtrlSym = std::popcount(dlCtrlMask);
    const uint32_t ulSym = std::popcount(ulMask);
    if (m_sampled)
    {
        return;
    }
    // The trace fires at slot start and reports the allocation for the slot that
    // is about to run, so close the previous interval first, then install this
    // slot's power. Same ordering the model uses; here it is rebuilt from scratch.
    Time now = Simulator::Now();
    m_analyticalJ += m_pendingPowerW * (now - m_lastSlotTime).GetSeconds();
    m_lastSlotTime = now;

    // sf = used REGs / (RBs in band * used symbols), TR 38.864 Section 5.1.
    auto sf = [availableRb](uint32_t reg, uint32_t sym) {
        return (availableRb > 0 && sym > 0)
                   ? std::min(1.0,
                              static_cast<double>(reg) / (static_cast<double>(availableRb) * sym))
                   : 0.0;
    };

    DoubleValue powerUnit;
    m_gnbModel->GetAttribute("PowerUnit", powerUnit);
    const double sa = 1.0;
    const double sp = m_gnbModel->GetSp();
    const double tSym = m_gnbModel->GetSymbolDuration().GetSeconds();
    const uint32_t symbolsPerSlot = m_gnbPhy->GetSymbolsPerSlot();
    const uint32_t scheduled = dlDataSym + dlCtrlSym + ulSym;
    const uint32_t idleSym = (scheduled < symbolsPerSlot) ? (symbolsPerSlot - scheduled) : 0;

    // TR 38.864 Section 5.2: sum the per-symbol powers over the slot. DL symbols
    // are charged at P_DL(sa, sf, sp), UL symbols at P_UL, unallocated at P3.
    double slotJ = 0.0;
    slotJ += dlDataSym * tSym * m_gnbModel->CalcDlPowerW(sa, sf(dlDataReg, dlDataSym), sp);
    slotJ += dlCtrlSym * tSym * m_gnbModel->CalcDlPowerW(sa, sf(dlCtrlReg, dlCtrlSym), sp);
    slotJ += ulSym * tSym * m_gnbModel->CalcUlPowerW(sa);
    slotJ += idleSym * tSym * powerUnit.Get() *
             m_gnbModel->GetRelativePower(NrGnbPowerState::MicroSleep);

    // The source integrates a constant current over the slot, so the slot energy
    // is exposed as its energy-equivalent average power (Biljana's point: this is
    // mathematically sufficient for BasicEnergySource).
    const double slotDurationS = symbolsPerSlot * tSym;
    m_pendingPowerW = (slotDurationS > 0.0) ? (slotJ / slotDurationS) : 0.0;
    m_slotEnergyKeys.insert(static_cast<uint64_t>(slotJ * 1e12));
}

void
NrEnergyIntegrationTestCase::UePowerChanged(double, double newPowerW)
{
    if (m_sampled)
    {
        return;
    }
    Time now = Simulator::Now();
    m_ueTraceJ += m_uePowerW * (now - m_ueLastTime).GetSeconds();
    m_ueLastTime = now;
    m_uePowerW = newPowerW;
}

void
NrEnergyIntegrationTestCase::RunScenario(bool enableDrx)
{
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0.0, 0.0, 10.0)); // gNB
    positions->Add(Vector(30.0, 0.0, 1.5)); // UE, close enough for a good MCS
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // One band, one CC, one BWP: the energy listeners drive BWP 0.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Configure the numerology before installing: the listeners query the PHY's
    // symbol period as soon as they attach, which is only valid once the PHY has
    // a slot period.
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(1));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    m_gnbPhy = NrHelper::GetGnbPhy(gnbDev.Get(0), 0);

    // ----- Internet stack, remote host and DL traffic -----

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDev));
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    nrHelper->AttachToGnb(ueDev.Get(0), gnbDev.Get(0));

    // ----- Energy stack, wired exactly as NrEnergyHelper wires it -----
    // Attached after AttachToGnb: the UE PHY only gets its numerology (and hence
    // a valid slot period, which the listener reads on attach) from the RRC
    // configuration that the attach produces.

    m_gnbSource = CreateObject<energy::BasicEnergySource>();
    m_gnbSource->SetInitialEnergy(INITIAL_ENERGY_J);
    m_gnbSource->SetSupplyVoltage(SUPPLY_VOLTAGE_V);
    m_gnbSource->SetNode(gnbNodes.Get(0));
    gnbNodes.Get(0)->AggregateObject(m_gnbSource);

    m_ueSource = CreateObject<energy::BasicEnergySource>();
    m_ueSource->SetInitialEnergy(INITIAL_ENERGY_J);
    m_ueSource->SetSupplyVoltage(SUPPLY_VOLTAGE_V);
    m_ueSource->SetNode(ueNodes.Get(0));
    ueNodes.Get(0)->AggregateObject(m_ueSource);

    m_gnbModel = CreateObject<NrGnbEnergyModel>();
    m_gnbModel->SetEnergySource(m_gnbSource);
    m_gnbSource->AppendDeviceEnergyModel(m_gnbModel);

    m_ueModel = CreateObject<NrUeEnergyModel>();
    m_ueModel->SetEnergySource(m_ueSource);
    m_ueSource->AppendDeviceEnergyModel(m_ueModel);

    Ptr<NrGnbPhyEnergyListener> gnbListener = CreateObject<NrGnbPhyEnergyListener>();
    gnbListener->SetEnergyModel(m_gnbModel);
    gnbListener->SetPhy(m_gnbPhy);
    gnbNodes.Get(0)->AggregateObject(gnbListener);

    Ptr<NrUePhyEnergyListener> ueListener = CreateObject<NrUePhyEnergyListener>();
    ueListener->SetEnergyModel(m_ueModel);
    ueListener->SetPhy(NrHelper::GetUePhy(ueDev.Get(0), 0));

    Ptr<NrUeDrxModel> drx;
    if (enableDrx)
    {
        drx = CreateObject<NrUeDrxModel>();
        drx->SetEnergyModel(m_ueModel);
        ueListener->SetDrxModel(drx);
        drx->Start(Simulator::Now());
        ueNodes.Get(0)->AggregateObject(drx);
    }
    ueNodes.Get(0)->AggregateObject(ueListener);

    // Initialize now so the models hold their t=0 power before anything runs;
    // both reconstructions below need that starting point.
    m_gnbModel->Initialize();
    m_ueModel->Initialize();
    m_pendingPowerW = m_gnbModel->GetCurrentPowerW();
    m_uePowerW = m_ueModel->GetCurrentPowerW();

    // Subscribe *after* the listener so the per-slot sp it refreshes is visible.
    m_gnbPhy->TraceConnectWithoutContext(
        "SlotEnergyStats",
        MakeCallback(&NrEnergyIntegrationTestCase::AnalyticalSlot, this));
    m_ueModel->TraceConnectWithoutContext(
        "InstantaneousPower",
        MakeCallback(&NrEnergyIntegrationTestCase::UePowerChanged, this));

    // Traffic runs only in the middle of the simulation, so the run contains
    // loaded slots, control-only slots and idle slots: different slot energies.
    const uint16_t dlPort = 1234;
    PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer serverApps = dlSink.Install(ueNodes.Get(0));

    UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(1200));
    dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(200)));
    ApplicationContainer clientApps = dlClient.Install(remoteHost);

    serverApps.Start(MilliSeconds(50));
    clientApps.Start(MilliSeconds(150));
    clientApps.Stop(MilliSeconds(300));

    // Every total must be sampled at the same instant. Reading them after
    // Simulator::Run() would not do: Now() is then the stop time, so the models
    // would project their open interval past the point the reconstructions closed.
    Simulator::Schedule(m_simTime, [this]() {
        m_analyticalJ += m_pendingPowerW * (Simulator::Now() - m_lastSlotTime).GetSeconds();
        m_ueTraceJ += m_uePowerW * (Simulator::Now() - m_ueLastTime).GetSeconds();
        m_gnbFinalModelJ = m_gnbModel->GetTotalEnergyJ();
        m_ueFinalModelJ = m_ueModel->GetTotalEnergyJ();
        m_gnbFinalDrainJ = INITIAL_ENERGY_J - m_gnbSource->GetRemainingEnergy();
        m_ueFinalDrainJ = INITIAL_ENERGY_J - m_ueSource->GetRemainingEnergy();
        m_sampled = true;
    });

    Simulator::Stop(m_simTime + MilliSeconds(1));
    Simulator::Run();
}

/**
 * @brief gNB: analytical == NrGnbEnergyModel::GetTotalEnergyJ() == source drain.
 */
class NrGnbEnergyIntegrationTestCase : public NrEnergyIntegrationTestCase
{
  public:
    NrGnbEnergyIntegrationTestCase()
        : NrEnergyIntegrationTestCase("gNB energy model and its BasicEnergySource agree")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyIntegrationTestCase::DoRun()
{
    RunScenario(false);

    const double modelJ = m_gnbFinalModelJ;
    const double drainedJ = m_gnbFinalDrainJ;

    // The scenario is only a meaningful test if the slots really differ.
    NS_TEST_ASSERT_MSG_GT(m_slotEnergyKeys.size(),
                          1,
                          "scenario produced identical slots; it cannot detect a slot-offset bug");
    NS_TEST_ASSERT_MSG_GT(modelJ, 0.0, "the gNB must consume energy");

    // Relative tolerance: the totals are large, and the source accumulates in
    // double precision over thousands of slots.
    NS_TEST_ASSERT_MSG_EQ_TOL(modelJ,
                              m_analyticalJ,
                              modelJ * REL_TOL,
                              "gNB model total differs from the analytical TR 38.864 energy");
    NS_TEST_ASSERT_MSG_EQ_TOL(drainedJ,
                              modelJ,
                              modelJ * REL_TOL,
                              "gNB source drain differs from the model total: the attached "
                              "EnergySource does not see what the model accounts");

    Simulator::Destroy();
}

/**
 * @brief UE: base + transient energy == NrUeEnergyModel::GetTotalEnergyJ() == drain.
 *
 * DRX is enabled so the UE actually enters sleep states and the TR 38.840
 * Table 19 transition energy is charged. That transition is the part an
 * EnergySource cannot see if it is added as a lump to the model's own total.
 */
class NrUeEnergyIntegrationTestCase : public NrEnergyIntegrationTestCase
{
  public:
    NrUeEnergyIntegrationTestCase()
        : NrEnergyIntegrationTestCase("UE energy model and its BasicEnergySource agree")
    {
    }

  private:
    void DoRun() override;
};

void
NrUeEnergyIntegrationTestCase::DoRun()
{
    RunScenario(true);

    const double modelJ = m_ueFinalModelJ;
    const double drainedJ = m_ueFinalDrainJ;

    NS_TEST_ASSERT_MSG_GT(modelJ, 0.0, "the UE must consume energy");
    NS_TEST_ASSERT_MSG_EQ_TOL(modelJ,
                              m_ueTraceJ,
                              modelJ * REL_TOL,
                              "UE model total differs from the integrated power trace "
                              "(base state energy plus transition transient)");
    NS_TEST_ASSERT_MSG_EQ_TOL(drainedJ,
                              modelJ,
                              modelJ * REL_TOL,
                              "UE source drain differs from the model total: the attached "
                              "EnergySource does not see what the model accounts");

    Simulator::Destroy();
}

/**
 * @brief A two-carrier gNB, end to end (TR 38.864 Section 5.1).
 *
 * The unit tests build carriers by hand. This one takes a real contiguous
 * two-CC band from CcBwpCreator, drives both carriers from live PHY traffic and
 * checks the three things that can only break once the pieces are assembled:
 *
 *   1. the 0.7 is derived from the REAL band layout, not from a test constant;
 *   2. the device energy is the weighted sum of what the carriers themselves
 *      accounted, so the weight is applied to every interval and not just to the
 *      instantaneous power;
 *   3. the attached EnergySource drained exactly that much, which is the
 *      property that fails if the per-carrier models are also appended to it.
 */
class NrGnbEnergyAggregatorIntegrationTestCase : public TestCase
{
  public:
    NrGnbEnergyAggregatorIntegrationTestCase()
        : TestCase("Two-carrier gNB: derived weights, aggregator and source agree")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyAggregatorIntegrationTestCase::DoRun()
{
    constexpr double INITIAL_J = 1.0e6;
    constexpr double VOLTAGE_V = 12.0;
    const Time SIM_TIME = MilliSeconds(200);

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 10.0));
    pos->Add(Vector(20.0, 0.0, 1.5));
    mobility.SetPositionAllocator(pos);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // TWO contiguous component carriers in one band - the topology the 0.7 rule
    // is about. CcBwpCreator lays them out edge to edge, which is exactly what
    // the aggregator's contiguity test looks for.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 40e6, 2);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NS_TEST_ASSERT_MSG_EQ(band.m_cc.size(), 2, "the band must really carry two CCs");

    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(1));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // ----- Internet, remote host, downlink traffic on both carriers -----

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDev));
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    nrHelper->AttachToClosestGnb(ueDev, gnbDev);

    UdpClientHelper client(ueIpIface.GetAddress(0), 1234);
    client.SetAttribute("MaxPackets", UintegerValue(100000));
    client.SetAttribute("Interval", TimeValue(MicroSeconds(200)));
    client.SetAttribute("PacketSize", UintegerValue(1000));
    ApplicationContainer clientApp = client.Install(remoteHost);
    clientApp.Start(MilliSeconds(20));
    clientApp.Stop(SIM_TIME);

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 1234));
    ApplicationContainer sinkApp = sink.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(0));
    sinkApp.Stop(SIM_TIME);

    // ----- One energy model per CARRIER, one aggregator for the device -----

    Ptr<energy::BasicEnergySource> source = CreateObject<energy::BasicEnergySource>();
    source->SetInitialEnergy(INITIAL_J);
    source->SetSupplyVoltage(VOLTAGE_V);
    source->SetNode(gnbNodes.Get(0));
    gnbNodes.Get(0)->AggregateObject(source);

    Ptr<NrGnbEnergyAggregator> aggregator = CreateObject<NrGnbEnergyAggregator>();
    std::vector<Ptr<NrGnbEnergyModel>> carriers;
    std::vector<Ptr<NrGnbPhyEnergyListener>> listeners;

    for (uint32_t cc = 0; cc < band.m_cc.size(); ++cc)
    {
        Ptr<NrGnbEnergyModel> model = CreateObject<NrGnbEnergyModel>();

        // The spectrum placement comes from the band itself, so the weight is
        // derived from the same numbers the PHY was configured with.
        aggregator->AddCarrier(model,
                               band.m_bandId,
                               band.m_cc[cc]->m_lowerFrequency,
                               band.m_cc[cc]->m_higherFrequency);

        Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
        listener->SetEnergyModel(model);
        listener->SetPhy(NrHelper::GetGnbPhy(gnbDev.Get(0), cc));

        // NOT AggregateObject(): a Node accepts only ONE object of a given
        // TypeId, so aggregating a second NrGnbPhyEnergyListener aborts. Holding
        // them here keeps every listener alive for the whole run instead. The
        // helper will need the same treatment once it wires more than one BWP.
        listeners.push_back(listener);
        carriers.push_back(model);
    }

    // ONLY the aggregator is appended: BasicEnergySource sums DoGetCurrentA()
    // unweighted, so appending the carriers too would drain the unweighted sum.
    aggregator->SetEnergySource(source);
    source->AppendDeviceEnergyModel(aggregator);
    aggregator->Initialize();

    // 1. The weights must come from the real band layout.
    NS_TEST_ASSERT_MSG_EQ(aggregator->GetNCarriers(), 2, "both carriers registered");
    NS_TEST_ASSERT_MSG_EQ_TOL(aggregator->GetCarrierWeight(0),
                              1.0,
                              1e-12,
                              "the lower CC of a contiguous pair is the anchor");
    NS_TEST_ASSERT_MSG_EQ_TOL(aggregator->GetCarrierWeight(1),
                              NrGnbEnergyAggregator::CONTIGUOUS_CC_SCALING,
                              1e-12,
                              "the additional contiguous CC is scaled by 0.7, derived from "
                              "the CcBwpCreator frequencies rather than hard-coded");

    Simulator::Stop(SIM_TIME);
    Simulator::Run();

    // Sample before Destroy(), while the simulation clock still holds SIM_TIME.
    const double aggregatorJ = aggregator->GetTotalEnergyJ();
    const double drainedJ = INITIAL_J - source->GetRemainingEnergy();

    // 2. Independent leg: each carrier accounted its own energy without knowing
    // about any weight, so the device total must be the weighted sum of those.
    double weightedCarriersJ = 0.0;
    for (uint32_t i = 0; i < carriers.size(); ++i)
    {
        const double carrierJ = carriers[i]->GetTotalEnergyJ();
        NS_TEST_ASSERT_MSG_GT(carrierJ, 0.0, "carrier " << i << " accounted no energy at all");
        weightedCarriersJ += aggregator->GetCarrierWeight(i) * carrierJ;
    }

    NS_TEST_ASSERT_MSG_EQ_TOL(aggregatorJ,
                              weightedCarriersJ,
                              1e-6,
                              "device energy must be the weighted sum of the carriers' own "
                              "accounting, over every interval and not just the final power");

    // 3. And the source must have drained exactly that.
    NS_TEST_ASSERT_MSG_EQ_TOL(drainedJ,
                              aggregatorJ,
                              1e-6,
                              "the EnergySource and the aggregator must account the same energy");

    // A guard against the whole thing passing while nothing happened: two idle
    // Cat 1 / Set 1 carriers over 200 ms already exceed this.
    NS_TEST_ASSERT_MSG_GT(aggregatorJ, 1.0, "the two-carrier gNB should have drawn real energy");

    // The unweighted sum is what a missing weight would produce; it must differ.
    double unweightedJ = 0.0;
    for (const auto& c : carriers)
    {
        unweightedJ += c->GetTotalEnergyJ();
    }
    NS_TEST_ASSERT_MSG_GT(std::abs(unweightedJ - aggregatorJ),
                          1.0,
                          "the 0.7 must actually change the device total");

    Simulator::Destroy();
}

/**
 * @brief NrEnergyHelper wires the whole stack, for one carrier and for several.
 *
 * The helper is the only place that sees the band topology, so it is the only
 * place that can decide how many energy models a device needs. This runs the
 * same scenario at one and at two component carriers and checks that the shape
 * of what comes back changes accordingly, that a multi-carrier device gets its
 * TR 38.864 weights, and that the source still agrees in both cases.
 */
class NrEnergyHelperInstallTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param numCc Component carriers to build the band with.
     * @param name  Test case name.
     */
    NrEnergyHelperInstallTestCase(uint8_t numCc, std::string name)
        : TestCase(name),
          m_numCc(numCc)
    {
    }

  private:
    void DoRun() override;
    uint8_t m_numCc; //!< Component carriers under test
};

void
NrEnergyHelperInstallTestCase::DoRun()
{
    constexpr double INITIAL_J = 1.0e6;
    const Time SIM_TIME = MilliSeconds(150);

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 10.0));
    pos->Add(Vector(20.0, 0.0, 1.5));
    mobility.SetPositionAllocator(pos);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 40e6, m_numCc);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(1));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>())
        ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDev));
    ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
        ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    nrHelper->AttachToClosestGnb(ueDev, gnbDev);

    UdpClientHelper client(ueIpIface.GetAddress(0), 1234);
    client.SetAttribute("MaxPackets", UintegerValue(100000));
    client.SetAttribute("Interval", TimeValue(MicroSeconds(200)));
    client.SetAttribute("PacketSize", UintegerValue(1000));
    ApplicationContainer clientApp = client.Install(remoteHost);
    clientApp.Start(MilliSeconds(20));
    clientApp.Stop(SIM_TIME);
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 1234));
    ApplicationContainer sinkApp = sink.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(0));
    sinkApp.Stop(SIM_TIME);

    // ----- the helper does all the energy wiring -----

    Ptr<energy::BasicEnergySource> source = CreateObject<energy::BasicEnergySource>();
    source->SetInitialEnergy(INITIAL_J);
    source->SetSupplyVoltage(12.0);
    source->SetNode(gnbNodes.Get(0));
    gnbNodes.Get(0)->AggregateObject(source);
    energy::EnergySourceContainer sources;
    sources.Add(source);

    NrEnergyHelper energyHelper;
    energy::DeviceEnergyModelContainer models = energyHelper.InstallGnb(gnbDev, sources, {band});

    NS_TEST_ASSERT_MSG_EQ(models.GetN(), 1, "one device model per gNB device");
    Ptr<NrGnbEnergyAggregator> agg = DynamicCast<NrGnbEnergyAggregator>(models.Get(0));
    // PeekPointer: the test macros stream their operands, and streaming a null
    // Ptr<> dereferences it, turning a clean failure into an abort.
    const bool isAggregator = (PeekPointer(agg) != nullptr);

    // EXPECT rather than ASSERT: a failing ASSERT returns from DoRun before
    // Simulator::Destroy(), and the next test case then starts on a live
    // simulator and aborts. These record the failure and let cleanup happen.
    if (m_numCc == 1)
    {
        NS_TEST_EXPECT_MSG_EQ(isAggregator,
                              false,
                              "a single-carrier device must not get an aggregator");
        NS_TEST_EXPECT_MSG_EQ(PeekPointer(DynamicCast<NrGnbEnergyModel>(models.Get(0))) != nullptr,
                              true,
                              "it gets the carrier model directly");
    }
    else
    {
        NS_TEST_EXPECT_MSG_EQ(isAggregator,
                              true,
                              "a multi-carrier device gets an aggregator: a helper that wired "
                              "only one bandwidth part would fall back to a single model");
        if (isAggregator)
        {
            NS_TEST_EXPECT_MSG_EQ(agg->GetNCarriers(), m_numCc, "one model per component carrier");
            NS_TEST_EXPECT_MSG_EQ_TOL(agg->GetCarrierWeight(0), 1.0, 1e-12, "anchor carrier");
            NS_TEST_EXPECT_MSG_EQ_TOL(agg->GetCarrierWeight(1),
                                      NrGnbEnergyAggregator::CONTIGUOUS_CC_SCALING,
                                      1e-12,
                                      "the helper's band data must yield the 0.7 for the "
                                      "second CC of a contiguous pair");
        }
    }

    Simulator::Stop(SIM_TIME);
    Simulator::Run();

    const double modelJ = models.Get(0)->GetTotalEnergyConsumption();
    const double drainedJ = INITIAL_J - source->GetRemainingEnergy();

    NS_TEST_ASSERT_MSG_GT(modelJ, 1.0, "the gNB should have drawn real energy");
    NS_TEST_ASSERT_MSG_EQ_TOL(drainedJ,
                              modelJ,
                              1e-6,
                              "whatever the helper built, the source and the device model "
                              "must still account the same energy");

    if (isAggregator)
    {
        // Every bandwidth part must actually be feeding its carrier: a helper
        // that wired only BWP 0 would leave the second carrier at its idle
        // baseline and still pass every assertion above.
        for (uint32_t cc = 0; cc < agg->GetNCarriers(); ++cc)
        {
            // A carrier with no listener is not zero-energy: it sits at the P3
            // baseline for the whole run. Compare against exactly that, which is
            // what a helper wiring only BWP 0 would leave behind.
            const double idleJ =
                agg->GetCarrier(cc)->CalcDlPowerW(0.0, 0.0, 0.0) * SIM_TIME.GetSeconds();
            NS_TEST_ASSERT_MSG_GT(agg->GetCarrier(cc)->GetTotalEnergyJ(),
                                  idleJ * 1.0001,
                                  "carrier " << cc
                                             << " never rose above its idle baseline, so no "
                                                "listener was wired to its bandwidth part");
        }
    }

    Simulator::Destroy();
}

/**
 * @brief Carriers in different frequency ranges take different 3GPP configs.
 *
 * TR 38.864 Table 5.1-1 defines separate reference configurations per frequency
 * range, and Table 5.1-3 keys P1..P5 off them, so a device whose carriers span
 * FR1 and FR2 cannot use one power table for both. This builds exactly that -
 * an FR1 band and an FR2 band on one gNB - gives each carrier its own
 * RefConfigSet through the helper, and checks the carriers really do sit at
 * different idle powers.
 */
class NrEnergyHelperMixedFrTestCase : public TestCase
{
  public:
    NrEnergyHelperMixedFrTestCase()
        : TestCase("NrEnergyHelper gives each carrier its own 3GPP configuration")
    {
    }

  private:
    void DoRun() override;
};

void
NrEnergyHelperMixedFrTestCase::DoRun()
{
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 10.0));
    pos->Add(Vector(15.0, 0.0, 1.5));
    mobility.SetPositionAllocator(pos);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Two bands: one FR1 carrier and one FR2 carrier on the same gNB. Separate
    // bands also mean these are NOT intra-band contiguous, so neither earns the
    // 0.7 - which the weights below confirm.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf fr1Conf(3.5e9, 40e6, 1);
    CcBwpCreator::SimpleOperationBandConf fr2Conf(28.0e9, 100e6, 1);
    OperationBandInfo fr1 = ccBwpCreator.CreateOperationBandContiguousCc(fr1Conf);
    OperationBandInfo fr2 = ccBwpCreator.CreateOperationBandContiguousCc(fr2Conf);
    channelHelper->AssignChannelsToBands({fr1, fr2});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({fr1, fr2});

    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Set1 is FR1 TDD 30 kHz, Set3 is FR2 TDD 120 kHz; the PHY numerology has to
    // match the set's subcarrier spacing or the model rejects it.
    NrHelper::GetGnbPhy(gnbDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(1));
    NrHelper::GetGnbPhy(gnbDev.Get(0), 1)->SetAttribute("Numerology", UintegerValue(3));

    Ptr<energy::BasicEnergySource> source = CreateObject<energy::BasicEnergySource>();
    source->SetInitialEnergy(1.0e6);
    source->SetSupplyVoltage(12.0);
    source->SetNode(gnbNodes.Get(0));
    gnbNodes.Get(0)->AggregateObject(source);
    energy::EnergySourceContainer sources;
    sources.Add(source);

    NrEnergyHelper energyHelper;
    energyHelper.SetGnbEnergyModelAttributeForCc(0, "RefConfigSet", StringValue("Set1"));
    energyHelper.SetGnbEnergyModelAttributeForCc(1, "RefConfigSet", StringValue("Set3"));
    energy::DeviceEnergyModelContainer models =
        energyHelper.InstallGnb(gnbDev, sources, {fr1, fr2});

    Ptr<NrGnbEnergyAggregator> agg = DynamicCast<NrGnbEnergyAggregator>(models.Get(0));
    const bool isAggregator = (PeekPointer(agg) != nullptr);
    NS_TEST_ASSERT_MSG_EQ(isAggregator, true, "two carriers must produce an aggregator");

    // Table 5.1-3 micro sleep: Set1 gives 55, Set3 gives 38. An idle carrier
    // sits exactly there, so the two carriers must differ - which is only true
    // if the per-carrier override actually reached the model.
    const double p1 = agg->GetCarrier(0)->CalcDlPowerW(0.0, 0.0, 0.0);
    const double p2 = agg->GetCarrier(1)->CalcDlPowerW(0.0, 0.0, 0.0);
    NS_TEST_ASSERT_MSG_EQ_TOL(p1, 55.0, 1e-9, "carrier 0 must use the FR1 Set1 table");
    NS_TEST_ASSERT_MSG_EQ_TOL(p2, 38.0, 1e-9, "carrier 1 must use the FR2 Set3 table");

    // Different bands, so no intra-band contiguity and no 0.7 for either.
    NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(0), 1.0, 1e-12, "inter-band: no discount");
    NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1), 1.0, 1e-12, "inter-band: no discount");

    Simulator::Destroy();
}

/**
 * @brief Energy framework integration test suite.
 */
class NrEnergyIntegrationTestSuite : public TestSuite
{
  public:
    NrEnergyIntegrationTestSuite()
        : TestSuite("nr-energy-integration", Type::SYSTEM)
    {
        AddTestCase(new NrGnbEnergyIntegrationTestCase(), Duration::QUICK);
        AddTestCase(new NrUeEnergyIntegrationTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyAggregatorIntegrationTestCase(), Duration::QUICK);
        AddTestCase(new NrEnergyHelperInstallTestCase(1, "NrEnergyHelper wires a one-carrier gNB"),
                    Duration::QUICK);
        AddTestCase(new NrEnergyHelperInstallTestCase(2, "NrEnergyHelper wires a two-carrier gNB"),
                    Duration::QUICK);
        AddTestCase(new NrEnergyHelperMixedFrTestCase(), Duration::QUICK);
    }
};

static NrEnergyIntegrationTestSuite g_nrEnergyIntegrationTestSuite; //!< the test suite

} // namespace ns3
