// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-unstructured-session.cc
 *
 * @brief Unstructured PDU session test suite (nr-unstructured-session). An unstructured
 * session (3GPP TS 23.501, Section 5.6.1) carries a single network layer protocol whose
 * payload the network does not parse, so these cases drive a protocol the model knows
 * nothing about, to show that the data plane never reads the packets it carries.
 *
 * The cases assert that a UE with no IP stack attaches and activates such a session, that
 * a packet crosses the whole core in both directions with its payload and its protocol
 * number intact, and that an unstructured session and an IP session run on the same UE
 * without either disturbing the other.
 */

#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"
#include "ns3/virtual-net-device.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrUnstructuredSessionTest");

namespace
{

/// A network layer protocol the model has no knowledge of, to prove the session is unstructured.
constexpr uint16_t DUMMY_PROTOCOL{0x88B7};

/// Records the packets a protocol handler receives, with the protocol they arrived with.
struct ProtocolSink
{
    /// One received packet
    struct Received
    {
        std::string payload;     ///< the payload the packet carried
        uint16_t protocolNumber; ///< the protocol the packet was delivered with
    };

    std::vector<Received> received; ///< every packet received so far

    /**
     * Handler to be registered on a node with Node::RegisterProtocolHandler().
     *
     * @param device the receiving device
     * @param packet the packet
     * @param protocolNumber the protocol the packet was delivered with
     * @param from the sender address
     * @param to the destination address
     * @param packetType the type of the packet
     */
    void Receive(Ptr<NetDevice> device,
                 Ptr<const Packet> packet,
                 uint16_t protocolNumber,
                 const Address& from,
                 const Address& to,
                 NetDevice::PacketType packetType)
    {
        std::string payload(packet->GetSize(), '\0');
        packet->CopyData(reinterpret_cast<uint8_t*>(payload.data()), packet->GetSize());
        received.push_back({payload, protocolNumber});
    }
};

/**
 * Build a packet carrying a text payload.
 *
 * @param payload the text to be carried
 * @return the packet
 */
Ptr<Packet>
MakePacket(const std::string& payload)
{
    return Create<Packet>(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
}

/**
 * A minimal NR topology with one gNB, one UE and one remote host behind the PGW, on which
 * the cases of this suite activate the sessions they need.
 *
 * The pieces are held by the fixture rather than by the test case so that the helpers
 * outlive the simulation.
 */
struct NrFixture
{
    Ptr<NrHelper> nrHelper;                   ///< the NR helper
    Ptr<NrPointToPointEpcHelper> nrEpcHelper; ///< the EPC helper
    Ptr<Node> pgw;                            ///< the PGW node
    NodeContainer ues;                        ///< the UE node
    NetDeviceContainer ueDevs;                ///< the UE device
    NetDeviceContainer gnbDevs;               ///< the gNB device
    BandwidthPartInfoPtrVector bwps;          ///< the bandwidth parts

    /**
     * Build the topology. The UE gets no IP stack: a case that wants one installs it
     * before attaching.
     */
    NrFixture()
    {
        nrHelper = CreateObject<NrHelper>();
        nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
        nrHelper->SetEpcHelper(nrEpcHelper);
        nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(0.0));
        nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(0.0));

        auto bandwidthAndBwpPair = nrHelper->CreateBandwidthParts({{2.8e9, 20e6, 1}}, "UMa");
        bwps = bandwidthAndBwpPair.second;

        pgw = nrEpcHelper->GetPgwNode();

        NodeContainer gnbs;
        gnbs.Create(1);
        ues.Create(1);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(gnbs);
        mobility.Install(ues);
        gnbs.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 10.0));
        ues.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(20.0, 0.0, 1.5));

        gnbDevs = nrHelper->InstallGnbDevice(gnbs, bwps);
        ueDevs = nrHelper->InstallUeDevice(ues, bwps);
    }

    /**
     * @return the IMSI of the UE
     */
    uint64_t GetImsi() const
    {
        return ueDevs.Get(0)->GetObject<NrUeNetDevice>()->GetImsi();
    }
};

} // namespace

/**
 * @ingroup tests
 *
 * @brief A UE with no IP stack attaches and activates an unstructured PDU session.
 *
 * The session carries unstructured payload and is reached through a device of its own, so it
 * identifies the UE by itself and needs no address on it. An IP session, whose packets
 * the core routes by address, is what requires the UE to have one.
 */
class NrUnstructuredActivationTestCase : public TestCase
{
  public:
    NrUnstructuredActivationTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredActivationTestCase::NrUnstructuredActivationTestCase()
    : TestCase("A UE with no IP stack activates an unstructured PDU session")
{
}

void
NrUnstructuredActivationTestCase::DoRun()
{
    NrFixture fx;

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    NS_TEST_ASSERT_MSG_NE(qfi, 0, "the session must be given a valid QFI");

    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto device = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(device, nullptr, "the session must have an egress device");
    NS_TEST_ASSERT_MSG_EQ(device->GetNode(), fx.pgw, "the egress device must live on the PGW node");

    Simulator::Stop(Seconds(0.5));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief A packet of an unstructured session crosses the core in both directions.
 *
 * The protocol driven here is unknown to the model, so nothing along the path can
 * recognize the payload: uplink is dispatched by the session the protocol was activated
 * for, and downlink recovers the protocol from the session the packet arrived on. The case
 * asserts both the payload and the protocol number at each end.
 */
class NrUnstructuredEndToEndTestCase : public TestCase
{
  public:
    NrUnstructuredEndToEndTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredEndToEndTestCase::NrUnstructuredEndToEndTestCase()
    : TestCase("An unstructured PDU session carries a packet across the core both ways")
{
}

void
NrUnstructuredEndToEndTestCase::DoRun()
{
    NrFixture fx;

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    ProtocolSink ueSink;
    ProtocolSink pgwSink;
    fx.ues.Get(0)->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &ueSink),
                                           DUMMY_PROTOCOL,
                                           fx.ueDevs.Get(0));
    fx.pgw->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &pgwSink),
                                    DUMMY_PROTOCOL,
                                    sessionDevice);

    const std::string uplinkPayload = "an unstructured payload going up";
    const std::string downlinkPayload = "an unstructured payload coming down";

    Simulator::Schedule(Seconds(0.3), [&]() {
        fx.ueDevs.Get(0)->Send(MakePacket(uplinkPayload),
                               fx.ueDevs.Get(0)->GetBroadcast(),
                               DUMMY_PROTOCOL);
    });
    Simulator::Schedule(Seconds(0.4), [&]() {
        sessionDevice->Send(MakePacket(downlinkPayload),
                            sessionDevice->GetBroadcast(),
                            DUMMY_PROTOCOL);
    });

    Simulator::Stop(Seconds(0.8));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.size(),
                          1,
                          "the uplink packet must come out of the session device");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).payload,
                          uplinkPayload,
                          "the uplink payload must cross the core unchanged");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the uplink packet must keep its protocol number");

    NS_TEST_ASSERT_MSG_EQ(ueSink.received.size(), 1, "the downlink packet must reach the UE");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).payload,
                          downlinkPayload,
                          "the downlink payload must cross the core unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the downlink protocol must be recovered from the session");
}

/**
 * @ingroup tests
 *
 * @brief A UE runs an IP session and an unstructured session at the same time.
 *
 * The UE has an IP stack, so it gets the default IP flow on attach, and it also activates
 * an unstructured session. UDP traffic and traffic of the unknown protocol run together,
 * and the case asserts that both arrive: the uplink dispatch tells them apart by protocol,
 * and the downlink by the session each arrived on.
 */
class NrUnstructuredDualStackTestCase : public TestCase
{
  public:
    NrUnstructuredDualStackTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredDualStackTestCase::NrUnstructuredDualStackTestCase()
    : TestCase("An IP session and an unstructured session run on the same UE")
{
}

void
NrUnstructuredDualStackTestCase::DoRun()
{
    NrFixture fx;

    // The remote host of the IP session, behind the PGW
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer internetDevices = p2ph.Install(fx.pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // The UE has an IP stack, so it also gets the default IP flow when it attaches
    internet.Install(fx.ues);
    Ipv4InterfaceContainer ueIpIface = fx.nrEpcHelper->AssignUeIpv4Address(fx.ueDevs);
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(fx.ues.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(fx.nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    ProtocolSink ueSink;
    ProtocolSink pgwSink;
    fx.ues.Get(0)->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &ueSink),
                                           DUMMY_PROTOCOL,
                                           fx.ueDevs.Get(0));
    fx.pgw->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &pgwSink),
                                    DUMMY_PROTOCOL,
                                    sessionDevice);

    // The IP session: a UDP echo between the UE and the remote host
    uint16_t echoPort = 4000;
    UdpEchoServerHelper echoServer(echoPort);
    ApplicationContainer serverApps = echoServer.Install(remoteHost);
    serverApps.Start(Seconds(0.1));

    UdpEchoClientHelper echoClient(internetIpIfaces.GetAddress(1), echoPort);
    echoClient.SetAttribute("MaxPackets", UintegerValue(4));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(50)));
    echoClient.SetAttribute("PacketSize", UintegerValue(100));
    ApplicationContainer clientApps = echoClient.Install(fx.ues.Get(0));
    clientApps.Start(Seconds(0.3));

    uint32_t echoReplies = 0;
    clientApps.Get(0)->TraceConnectWithoutContext(
        "Rx",
        MakeCallback(+[](uint32_t* count, Ptr<const Packet>) { ++(*count); }).Bind(&echoReplies));

    // The unstructured session, running at the same time as the UDP echo
    const std::string uplinkPayload = "unstructured traffic beside IP";
    const std::string downlinkPayload = "unstructured answer beside IP";
    Simulator::Schedule(Seconds(0.32), [&]() {
        fx.ueDevs.Get(0)->Send(MakePacket(uplinkPayload),
                               fx.ueDevs.Get(0)->GetBroadcast(),
                               DUMMY_PROTOCOL);
    });
    Simulator::Schedule(Seconds(0.42), [&]() {
        sessionDevice->Send(MakePacket(downlinkPayload),
                            sessionDevice->GetBroadcast(),
                            DUMMY_PROTOCOL);
    });

    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_GT(echoReplies, 0, "the IP session must carry the UDP echo");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.size(),
                          1,
                          "the unstructured uplink must arrive while IP traffic runs");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).payload,
                          uplinkPayload,
                          "the unstructured uplink payload must be unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.size(),
                          1,
                          "the unstructured downlink must arrive while IP traffic runs");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).payload,
                          downlinkPayload,
                          "the unstructured downlink payload must be unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the IP session must not disturb the unstructured one");
}

/**
 * @ingroup tests
 *
 * @brief Unstructured PDU session test suite
 */
class NrUnstructuredSessionTestSuite : public TestSuite
{
  public:
    NrUnstructuredSessionTestSuite();
};

NrUnstructuredSessionTestSuite::NrUnstructuredSessionTestSuite()
    : TestSuite("nr-unstructured-session", Type::SYSTEM)
{
    AddTestCase(new NrUnstructuredActivationTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredEndToEndTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredDualStackTestCase(), Duration::QUICK);
}

/// Static variable for test initialization
static NrUnstructuredSessionTestSuite g_nrUnstructuredSessionTestSuite;
