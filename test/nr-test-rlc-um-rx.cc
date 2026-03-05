// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-test-entities.h"

#include "ns3/log.h"
#include "ns3/nr-rlc-header.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/nr-rlc-um.h"
#include "ns3/packet.h"

NS_LOG_COMPONENT_DEFINE("NrRlcUmTestCase");

namespace ns3
{

class NrRlcUmTestCase : public TestCase
{
  public:
    NrRlcUmTestCase()
        : TestCase("Test RLC UM RX: Check packets not reassembled when window advances")
    {
    }

  private:
    void DoRun() override;

    /**
     * Create an RLC PDU
     *
     * @param payloadSize Size of the RLC PDU payload in bytes
     * @param rlcSn RLC Sequence Number (SN) of the PDU
     * @param extensionBit Extension Bit flag:
     *                     0 = no more Length Indicators (LI) follow;
     *                     1 = at least one more LI follows
     *                     Default: DATA_FIELD_FOLLOWS (LI = 0)
     * @param framingInfo Framing Info (FI) flag, indicates SDU boundaries:
     *                    00 = starts and ends with an SDU
     *                    01 = starts with an SDU but does not end with one
     *                    10 = does not start with an SDU but ends with one
     *                    11 = neither starts nor ends with an SDU
     *                    Default: FIRST_BYTE | LAST_BYTE (FI = 00)
     * @return A Ptr<Packet> representing the RLC PDU
     */
    Ptr<Packet> CreateRlcPdu(uint32_t payloadSize,
                             uint16_t rlcSn,
                             uint8_t extensionBit = NrRlcHeader::DATA_FIELD_FOLLOWS,
                             uint8_t framingInfo = NrRlcHeader::FIRST_BYTE |
                                                   NrRlcHeader::LAST_BYTE);

    Ptr<NrTestPdcp> rxPdcp; ///< The reception PDCP
};

Ptr<Packet>
NrRlcUmTestCase::CreateRlcPdu(uint32_t payloadSize,
                              uint16_t rlcSn,
                              uint8_t extensionBit,
                              uint8_t framingInfo)
{
    // Create dummy payload of given size
    Ptr<Packet> rlcPdu = Create<Packet>(payloadSize);

    // Create RLC header
    NrRlcHeader rlcHeader;
    rlcHeader.SetSequenceNumber(nr::SequenceNumber10(rlcSn));
    rlcHeader.PushExtensionBit(extensionBit);
    rlcHeader.SetFramingInfo(framingInfo);

    // Add header to packet
    rlcPdu->AddHeader(rlcHeader);

    return rlcPdu;
}

void
NrRlcUmTestCase::DoRun()
{
    auto logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);
    LogComponentEnable("NrRlcUmTestCase", logLevel);
    LogComponentEnable("NrRlcUm", logLevel);

    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("DoRun started");

    // Instantiate RLC
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();

    // Create transmission PDCP test entity
    rxPdcp = CreateObject<NrTestPdcp>();

    // Connect RLC with Test PDCP
    rlc->SetNrRlcSapUser(rxPdcp->GetNrRlcSapUser());

    NS_LOG_INFO("Step 1: Initialized m_rxBuffer with SN 0, 502, 1016, 1017, 1018");
    rlc->m_rxBuffer[0] = CreateRlcPdu(10, 0, 0x00, NrRlcHeader::FIRST_BYTE);
    rlc->m_rxBuffer[1016] = CreateRlcPdu(20, 1016, 0x00, NrRlcHeader::FIRST_BYTE);
    rlc->m_rxBuffer[1017] =
        CreateRlcPdu(15, 1017, 0x00, NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE);
    rlc->m_rxBuffer[1018] =
        CreateRlcPdu(15, 1018, 0x00, NrRlcHeader::NO_FIRST_BYTE | NrRlcHeader::LAST_BYTE);
    rlc->m_rxBuffer[502] = CreateRlcPdu(30, 502, 0x00, NrRlcHeader::FIRST_BYTE);

    NS_LOG_INFO(
        "Step 2: Reception window set (VR(UR)=1015, VR(UH)=503, windowSize=512) -> [1015, 503)");
    rlc->m_vrUr = 1015;
    rlc->m_vrUx = 0;
    rlc->m_vrUh = 503;
    rlc->m_windowSize = 512;

    rlc->m_expectedSeqNumber = 1015;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    // Create RLC PDU with SN=505, FI = 00, E = 0
    NS_LOG_INFO("Step 3: Creating PDU with SN=505");
    Ptr<Packet> pdu = CreateRlcPdu(50, 505, 0x00, NrRlcHeader::FIRST_BYTE);

    // Add RLC packet tag
    NrRlcTag rlcTag(Simulator::Now());
    pdu->AddByteTag(rlcTag);

    NrMacSapUser::ReceivePduParameters rxPduParams = {};
    rxPduParams.p = pdu;
    rxPduParams.lcid = 0;
    rxPduParams.rnti = 0;

    NS_LOG_INFO("Step 4: Calling DoReceivePdu with SN=505");
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    rlc->DoReceivePdu(rxPduParams);

    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("Step 5: DoReceivePdu finished");

    // Bug check: Old code leaves SN {1016, 1017} in m_rxBuffer un-reassembled.
    // Updated code (MR !354) reassembles them before advancing VR(UR) to 1018.
    for (const auto& entry : rlc->m_rxBuffer)
    {
        uint16_t sn = entry.first;
        uint16_t vrUr = rlc->m_vrUr.GetValue();
        uint16_t vrUh = rlc->m_vrUh.GetValue();

        bool insideWindow;

        if (vrUr <= vrUh)
        {
            // No wrap
            insideWindow = (sn >= vrUr && sn < vrUh);
        }
        else
        {
            // Wrap-around
            insideWindow = (sn >= vrUr || sn < vrUh);
        }

        if (!insideWindow)
        {
            NS_LOG_ERROR(
                " ERROR: Packet stored outside reception window, SN=" + std::to_string(sn) +
                ", [VR(UR)=" + std::to_string(vrUr) + ", VR(UH)=" + std::to_string(vrUh) + ")");

            // Trigger the assert
            NS_TEST_ASSERT_MSG_EQ(false, true, "");
        }
    }

    NS_LOG_INFO("m_rxBuffer key (SN) contents after DoReceivePdu:");
    for (const auto& entry : rlc->m_rxBuffer)
    {
        NS_LOG_INFO("SN = " << entry.first);
    }
}

class NrRlcUmTestSuite : public TestSuite
{
  public:
    NrRlcUmTestSuite()
        : TestSuite("nr-test-rlc-um-rx", Type::SYSTEM)
    {
        AddTestCase(new NrRlcUmTestCase(), Duration::QUICK);
    }
};

static NrRlcUmTestSuite g_nrRlcUmTestSuite;

} // namespace ns3
