/*
 * Copyright (c) 2013 Magister Solutions
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *         Alexander Krotov <krotov@iitp.ru>
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
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverDelayTest");

/**
 * @ingroup nr-test
 *
 * @brief Verifying that the time needed for handover is under a
 * specified threshold.
 */

class NrHandoverDelayTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param numberOfComponentCarriers number of component carriers
     * @param useIdealRrc if true, use the ideal RRC
     * @param handoverTime the time of handover
     * @param delayThreshold the delay threshold
     * @param simulationDuration duration of the simulation
     */
    NrHandoverDelayTestCase(uint8_t numberOfComponentCarriers,
                            bool useIdealRrc,
                            Time handoverTime,
                            Time delayThreshold,
                            Time simulationDuration)
        : TestCase("Verifying that the time needed for handover is under a specified threshold"),
          m_numberOfComponentCarriers(numberOfComponentCarriers),
          m_useIdealRrc(useIdealRrc),
          m_handoverTime(handoverTime),
          m_delayThreshold(delayThreshold),
          m_simulationDuration(simulationDuration),
          m_ueHandoverStart(),
          m_gnbHandoverStart()
    {
    }

  private:
    void DoRun() override;

    /**
     * UE handover start callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void UeHandoverStartCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellid,
                                 uint16_t rnti,
                                 uint16_t targetCellId);
    /**
     * UE handover end OK callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     */
    void UeHandoverEndOkCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellid,
                                 uint16_t rnti);
    /**
     * gNB handover start callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void GnbHandoverStartCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellid,
                                  uint16_t rnti,
                                  uint16_t targetCellId);
    /**
     * gNB handover end OK callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     */
    void GnbHandoverEndOkCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellid,
                                  uint16_t rnti);

    uint8_t m_numberOfComponentCarriers; ///< Number of component carriers
    bool m_useIdealRrc;                  ///< use ideal RRC?
    Time m_handoverTime;                 ///< handover time
    Time m_delayThreshold;               ///< the delay threshold
    Time m_simulationDuration;           ///< the simulation duration

    Time m_ueHandoverStart;     ///< UE handover start time
    Time m_gnbHandoverStart;    ///< gNB handover start time
    bool m_handoverDone{false}; ///< Flag indicated whether the handover happened
};

void
NrHandoverDelayTestCase::DoRun()
{
    NS_LOG_INFO("-----test case: ideal RRC = " << m_useIdealRrc << " handover time = "
                                               << m_handoverTime.As(Time::S) << "-----");

    /*
     * Helpers.
     */
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    /*
     * Physical layer.
     *
     * gNB 0                    UE                      gNB 1
     *
     *    x ----------------------- x ----------------------- x
     *              500 m                      500 m
     */
    // Create nodes.
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    auto ueNode = CreateObject<Node>();

    // Setup mobility
    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(1000, 0, 0));
    posAlloc->Add(Vector(500, 0, 0));

    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure LogDistance propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(
        2.8e9,
        20e6,
        static_cast<uint8_t>(m_numberOfComponentCarriers));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    /*
     * Link layer.
     */
    auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);

    /*
     * Network layer.
     */
    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs;
    ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    // Setup traces.
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NrHandoverDelayTestCase::UeHandoverStartCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverDelayTestCase::UeHandoverEndOkCallback, this));

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                    MakeCallback(&NrHandoverDelayTestCase::GnbHandoverStartCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverDelayTestCase::GnbHandoverEndOkCallback, this));

    // Prepare handover.
    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));
    nrHelper->HandoverRequest(m_handoverTime, ueDev, gnbDevs.Get(0), gnbDevs.Get(1));

    nrEpcHelper->AssignStreams(0);
    inetStackHelper.AssignStreams(gnbNodes, 1000);
    inetStackHelper.AssignStreams(ueNode, 2000);
    nrHelper->AssignStreams(gnbDevs, 3000);
    nrHelper->AssignStreams(ueDev, 4000);

    // Run simulation.
    Simulator::Stop(m_simulationDuration);
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_EXPECT_MSG_EQ(m_handoverDone, true, "Expected a handover");
}

void
NrHandoverDelayTestCase::UeHandoverStartCallback(std::string context,
                                                 uint64_t imsi,
                                                 uint16_t cellid,
                                                 uint16_t rnti,
                                                 uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context);
    m_ueHandoverStart = Simulator::Now();
}

void
NrHandoverDelayTestCase::UeHandoverEndOkCallback(std::string context,
                                                 uint64_t imsi,
                                                 uint16_t cellid,
                                                 uint16_t rnti)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(m_ueHandoverStart.IsStrictlyPositive());
    Time delay = Simulator::Now() - m_ueHandoverStart;

    NS_LOG_DEBUG(this << " UE delay = " << delay.As(Time::S));
    NS_TEST_ASSERT_MSG_LT(delay,
                          m_delayThreshold,
                          "UE handover delay is higher than the allowed threshold "
                              << "(ideal RRC = " << m_useIdealRrc
                              << " handover time = " << m_handoverTime.As(Time::S) << ")");
    m_handoverDone = true;
}

void
NrHandoverDelayTestCase::GnbHandoverStartCallback(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t cellid,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context);
    NS_TEST_EXPECT_MSG_NE(cellid, targetCellId, "Expected handover to different cell");
    m_gnbHandoverStart = Simulator::Now();
}

void
NrHandoverDelayTestCase::GnbHandoverEndOkCallback(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t cellid,
                                                  uint16_t rnti)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(m_gnbHandoverStart.IsStrictlyPositive());
    Time delay = Simulator::Now() - m_gnbHandoverStart;

    NS_LOG_DEBUG(this << " gNB delay = " << delay.As(Time::S));
    NS_TEST_ASSERT_MSG_LT(delay,
                          m_delayThreshold,
                          "gNB handover delay is higher than the allowed threshold "
                              << "(ideal RRC = " << m_useIdealRrc
                              << " handover time = " << m_handoverTime.As(Time::S) << ")");
}

/**
 * @ingroup nr-test
 *
 * @brief NR Handover Delay Test Suite
 */

static class NrHandoverDelayTestSuite : public TestSuite
{
  public:
    NrHandoverDelayTestSuite()
        : TestSuite("nr-handover-delay", Type::SYSTEM)
    {
        // LogComponentEnable ("NrHandoverDelayTest", LOG_PREFIX_TIME);
        // LogComponentEnable ("NrHandoverDelayTest", LOG_DEBUG);
        // LogComponentEnable ("NrHandoverDelayTest", LOG_INFO);

        // HANDOVER DELAY TEST CASES WITH IDEAL RRC (THRESHOLD = 0.005 sec)

        for (Time handoverTime = Seconds(0.100); handoverTime < Seconds(0.110);
             handoverTime += Seconds(0.001))
        {
            // arguments: useIdealRrc, handoverTime, delayThreshold, simulationDuration
            AddTestCase(
                new NrHandoverDelayTestCase(1, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(2, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(4, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
        }

        // HANDOVER DELAY TEST CASES WITH REAL RRC (THRESHOLD = 0.020 sec)
        /* todo: re-enable when we have real RRC
        for (Time handoverTime = Seconds(0.100); handoverTime < Seconds(0.110);
             handoverTime += Seconds(0.001))
        {
            // arguments: useIdealRrc, handoverTime, delayThreshold, simulationDuration
            AddTestCase(
                new NrHandoverDelayTestCase(1, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(2, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(4, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
        }
        */
    }
} g_nrHandoverDelayTestSuite; ///< the test suite
