/*
 * Copyright (c) 2013 Magister Solutions (original test-lte-handover-delay.cc)
 * Copyright (c) 2021 University of Washington (handover failure cases)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Sachin Nayak <sachinnn@uw.edu>
 */

#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/data-rate.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverFailureTest");

/**
 * @ingroup nr-test
 *
 * @brief Verifying that a handover failure occurs due to various causes
 *
 * Handover failure cases dealt with in this test include the below.
 *
 * 1. Handover failure due to max random access channel (RACH) attempts from UE to target gNB
 * 2. Handover failure due to non-allocation of non-contention preamble to UE at target gNB
 * 3. Handover failure due to HANDOVER JOINING timeout (3 cases)
 * 4. Handover failure due to HANDOVER LEAVING timeout (3 cases)
 *
 * \sa ns3::NrHandoverFailureTestCase
 */
class NrHandoverFailureTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the name of the test case, to be displayed in the test result
     * @param useIdealRrc if true, use the ideal RRC
     * @param handoverTime the time of handover
     * @param simulationDuration duration of the simulation
     * @param numberOfRaPreambles number of random access preambles available for contention based
     RACH process
     *                            number of non-contention preambles available for handover = (64 -
     numberRaPreambles)
     *                            as numberOfRaPreambles out of the max 64 are reserved contention
     based RACH process
     * @param preambleTransMax maximum number of random access preamble transmissions from UE to
     gNB
     * @param raResponseWindowSize window length for reception of random access response (RAR)
     * @param handoverJoiningTimeout time before which RRC RECONFIGURATION COMPLETE must be received
                                     at target gNB after it receives a handover request
                                     Else, the UE context is destroyed in the RRC.
                                     Timeout can occur before different stages as below.
                                     i. Reception of RRC CONNECTION RECONFIGURATION at source gNB
                                     ii. Non-contention random access procedure from UE to target
     gNB iii. Reception of RRC CONNECTION RECONFIGURATION COMPLETE at target gNB
     * @param handoverLeavingTimeout time before which source gNB must receive a UE context
     release from target gNB or RRC CONNECTION RESTABLISHMENT from UE after issuing a handover
     request Else, the UE context is destroyed in the RRC. Timeout can occur before any of the cases
     in HANDOVER JOINING TIMEOUT
     * @param targetGnbPosition position of the target gNB
     */
    NrHandoverFailureTestCase(std::string name,
                              bool useIdealRrc,
                              Time handoverTime,
                              Time simulationDuration,
                              uint8_t numberOfRaPreambles,
                              uint8_t preambleTransMax,
                              uint8_t raResponseWindowSize,
                              Time handoverJoiningTimeout,
                              Time handoverLeavingTimeout,
                              uint16_t targetGnbPosition)
        : TestCase(name),
          m_useIdealRrc(useIdealRrc),
          m_handoverTime(handoverTime),
          m_simulationDuration(simulationDuration),
          m_numberOfRaPreambles(numberOfRaPreambles),
          m_preambleTransMax(preambleTransMax),
          m_raResponseWindowSize(raResponseWindowSize),
          m_handoverJoiningTimeout(handoverJoiningTimeout),
          m_handoverLeavingTimeout(handoverLeavingTimeout),
          m_targetGnbPosition(targetGnbPosition),
          m_hasHandoverFailureOccurred(false)
    {
    }

  private:
    /**
     * @brief Run a simulation of a two gNB network using the parameters
     *        provided to the constructor function.
     */
    void DoRun() override;

    /**
     * @brief Called at the end of simulation and verifies that a handover
     *        and a handover failure has occurred in the simulation.
     */
    void DoTeardown() override;

    /**
     * UE handover start callback function to indicate start of handover
     * @param context the context string
     * @param imsi the IMSI
     * @param sourceCellId the source cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void UeHandoverStartCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t sourceCellId,
                                 uint16_t rnti,
                                 uint16_t targetCellId);

    /**
     * Handover failure callback due to maximum RACH transmissions reached from UE to target gNB
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverFailureMaxRach(std::string context,
                                uint64_t imsi,
                                uint16_t rnti,
                                uint16_t targetCellId);

    /**
     * Handover failure callback due to non-allocation of non-contention preamble at target gNB
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverFailureNoPreamble(std::string context,
                                   uint64_t imsi,
                                   uint16_t rnti,
                                   uint16_t targetCellId);

    /**
     * Handover failure callback due to handover joining timeout at target gNB
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverFailureJoining(std::string context,
                                uint64_t imsi,
                                uint16_t rnti,
                                uint16_t targetCellId);

    /**
     * Handover failure callback due to handover leaving timeout at source gNB
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverFailureLeaving(std::string context,
                                uint64_t imsi,
                                uint16_t rnti,
                                uint16_t targetCellId);

    bool m_useIdealRrc;             ///< use ideal RRC?
    Time m_handoverTime;            ///< handover time
    Time m_simulationDuration;      ///< the simulation duration
    uint8_t m_numberOfRaPreambles;  ///< number of random access preambles for contention based RACH
                                    ///< process
    uint8_t m_preambleTransMax;     ///< max number of RACH preambles possible from UE to gNB
    uint8_t m_raResponseWindowSize; ///< window length for reception of RAR
    Time m_handoverJoiningTimeout;  ///< handover joining timeout duration at target gNB
    Time m_handoverLeavingTimeout;  ///< handover leaving timeout duration at source gNB
    uint16_t m_targetGnbPosition;   ///< position of the target gNB
    bool m_hasHandoverFailureOccurred; ///< has handover failure occurred in simulation

    // end of class NrHandoverFailureTestCase
};

void
NrHandoverFailureTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());
    uint32_t previousSeed = RngSeedManager::GetSeed();
    uint64_t previousRun = RngSeedManager::GetRun();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(2);

    /*
     * Helpers.
     */
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Set parameters for helpers based on the test case parameters.
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));
    Config::SetDefault("ns3::NrGnbMac::NumberOfRaPreambles", UintegerValue(m_numberOfRaPreambles));
    Config::SetDefault("ns3::NrGnbMac::PreambleTransMax", UintegerValue(m_preambleTransMax));
    Config::SetDefault("ns3::NrGnbMac::RaResponseWindowSize",
                       UintegerValue(m_raResponseWindowSize));
    Config::SetDefault("ns3::NrGnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(m_handoverJoiningTimeout));
    Config::SetDefault("ns3::NrGnbRrc::HandoverLeavingTimeoutDuration",
                       TimeValue(m_handoverLeavingTimeout));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure LogDistance propagation loss model before assign it to band
    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    /*
     * Physical layer.
     *
     * gNB 0                    UE                         gNB 1
     *
     *    x ----------------------- x -------------------------- x
     *              200 m               m_targetGnbPosition
     *  source                                                 target
     */
    // Create nodes.
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    auto ueNode = CreateObject<Node>();

    // Setup mobility
    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(m_targetGnbPosition, 0, 0));
    posAlloc->Add(Vector(200, 0, 0));

    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    /*
     * Link layer.
     */
    auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);
    auto castedUeDev = DynamicCast<NrUeNetDevice>(ueDev);

    /*
     * Network layer.
     */
    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs;
    ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    // Setup traces.
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NrHandoverFailureTestCase::UeHandoverStartCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&NrHandoverFailureTestCase::HandoverFailureMaxRach, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&NrHandoverFailureTestCase::HandoverFailureNoPreamble, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureJoining",
                    MakeCallback(&NrHandoverFailureTestCase::HandoverFailureJoining, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureLeaving",
                    MakeCallback(&NrHandoverFailureTestCase::HandoverFailureLeaving, this));

    // Prepare handover.
    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));
    nrHelper->HandoverRequest(m_handoverTime, ueDev, gnbDevs.Get(0), gnbDevs.Get(1));

    nrEpcHelper->AssignStreams(0);
    inetStackHelper.AssignStreams(gnbNodes, 1000);
    inetStackHelper.AssignStreams(ueNode, 2000);
    nrHelper->AssignStreams(gnbDevs, 3000);
    nrHelper->AssignStreams(ueDev, 4000);

    // Forcefully drop RACH preambles to reach m_preambleTransMax, since we do not have an error
    // model for the control channel
    if (m_preambleTransMax == 3)
    {
        auto gnbTargetDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(1))->GetPhy(0);
        Simulator::Schedule(m_handoverTime, [gnbTargetDev]() {
            gnbTargetDev->SetAttribute("TestDropRachPreambles", BooleanValue(true));
        });
    }

    // Run simulation.
    Simulator::Stop(m_simulationDuration);
    Simulator::Run();
    Simulator::Destroy();

    RngSeedManager::SetSeed(previousSeed);
    RngSeedManager::SetRun(previousRun);
}

void
NrHandoverFailureTestCase::UeHandoverStartCallback(std::string context,
                                                   uint64_t imsi,
                                                   uint16_t sourceCellId,
                                                   uint16_t rnti,
                                                   uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << " " << context << " IMSI-" << imsi << " sourceCellID-" << sourceCellId
                         << " RNTI-" << rnti << " targetCellID-" << targetCellId);
    NS_LOG_INFO("HANDOVER COMMAND received through at UE "
                << imsi << " to handover from " << sourceCellId << " to " << targetCellId);
}

void
NrHandoverFailureTestCase::HandoverFailureMaxRach(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << rnti << targetCellId);
    m_hasHandoverFailureOccurred = true;
}

void
NrHandoverFailureTestCase::HandoverFailureNoPreamble(std::string context,
                                                     uint64_t imsi,
                                                     uint16_t rnti,
                                                     uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << rnti << targetCellId);
    m_hasHandoverFailureOccurred = true;
}

void
NrHandoverFailureTestCase::HandoverFailureJoining(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << rnti << targetCellId);
    m_hasHandoverFailureOccurred = true;
}

void
NrHandoverFailureTestCase::HandoverFailureLeaving(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << rnti << targetCellId);
    m_hasHandoverFailureOccurred = true;
}

void
NrHandoverFailureTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    NS_TEST_ASSERT_MSG_EQ(m_hasHandoverFailureOccurred, true, "Handover failure did not occur");
}

/**
 * @ingroup nr-test
 *
 * The following log components can be used to debug this test's behavior:
 * NrHandoverFailureTest:NrGnbRrc:NrGnbMac:NrUeRrc:NrEpcX2
 *
 * @brief NR Handover Failure Test Suite
 */
static class NrHandoverFailureTestSuite : public TestSuite
{
  public:
    NrHandoverFailureTestSuite()
        : TestSuite("nr-handover-failure", Type::SYSTEM)
    {
        // Argument sequence for all test cases: useIdealRrc, handoverTime, simulationDuration,
        // numberOfRaPreambles, preambleTransMax, raResponseWindowSize,
        //                                       handoverJoiningTimeout, handoverLeavingTimeout

        // Test cases for REAL RRC protocol
        /* todo: re-enable when real RRC is working
        AddTestCase(new NrHandoverFailureTestCase("REAL Handover failure due to maximum RACH "
                                                  "transmissions reached from UE to target gNB",
                                                  false,
                                                  Seconds(0.200),
                                                  Seconds(0.300),
                                                  52,
                                                  3,
                                                  3,
                                                  MilliSeconds(200),
                                                  MilliSeconds(500),
                                                  2500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to non-allocation of non-contention preamble at "
                        "target gNB due to max number reached",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        64,
                        50,
                        3,
                        MilliSeconds(200),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER JOINING timeout before reception of "
                        "RRC CONNECTION RECONFIGURATION at source gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(0),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER JOINING timeout before completion "
                        "of non-contention RACH process to target gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(15),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER JOINING timeout before reception of "
                        "RRC CONNECTION RECONFIGURATION COMPLETE at target gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(18),
                        MilliSeconds(500),
                        500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER LEAVING timeout before reception of "
                        "RRC CONNECTION RECONFIGURATION at source gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(200),
                        MilliSeconds(0),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER LEAVING timeout before completion "
                        "of non-contention RACH process to target gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(200),
                        MilliSeconds(15),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "REAL Handover failure due to HANDOVER LEAVING timeout before reception of "
                        "RRC CONNECTION RECONFIGURATION COMPLETE at target gNB",
                        false,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(200),
                        MilliSeconds(18),
                        500),
                    TestCase::Duration::QUICK);
        */
        // Test cases for IDEAL RRC protocol
        AddTestCase(new NrHandoverFailureTestCase("IDEAL Handover failure due to maximum RACH "
                                                  "transmissions reached from UE to target gNB",
                                                  true,
                                                  Seconds(0.100),
                                                  Seconds(0.200),
                                                  52,
                                                  3,
                                                  3,
                                                  MilliSeconds(200),
                                                  MilliSeconds(500),
                                                  1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to non-allocation of non-contention preamble "
                        "at target gNB due to max number reached",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        64,
                        50,
                        3,
                        MilliSeconds(200),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER JOINING timeout before reception "
                        "of RRC CONNECTION RECONFIGURATION at source gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(0),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER JOINING timeout before completion "
                        "of non-contention RACH process to target gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(4),
                        MilliSeconds(500),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER JOINING timeout before reception "
                        "of RRC CONNECTION RECONFIGURATION COMPLETE at target gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(4),
                        MilliSeconds(500),
                        500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER LEAVING timeout before reception "
                        "of RRC CONNECTION RECONFIGURATION at source gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(500),
                        MilliSeconds(0),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER LEAVING timeout before completion "
                        "of non-contention RACH process to target gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(500),
                        MilliSeconds(4),
                        1500),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverFailureTestCase(
                        "IDEAL Handover failure due to HANDOVER LEAVING timeout before reception "
                        "of RRC CONNECTION RECONFIGURATION COMPLETE at target gNB",
                        true,
                        Seconds(0.100),
                        Seconds(0.200),
                        52,
                        50,
                        3,
                        MilliSeconds(500),
                        MilliSeconds(4),
                        500),
                    TestCase::Duration::QUICK);
    }
} g_nrHandoverFailureTestSuite; ///< end of NrHandoverFailureTestSuite ()
