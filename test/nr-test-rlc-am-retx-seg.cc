/*
 * Test for NrRlcAm retransmission re-segmentation (TS 36.322 section 5.2.1):
 * a NACKed AMD PDU larger than the granted Tx opportunity must be sent as a
 * series of segments, and the receiver must reassemble them into the original
 * PDU before SDU reassembly/delivery.
 */

#include "ns3/nr-mac-sap.h"
#include "ns3/nr-rlc-am-header.h"
#include "ns3/nr-rlc-am.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRlcAmRetxSegTest");

namespace
{

/**
 * Minimal RLC SAP user: concatenates every delivered SDU payload.
 */
class RetxSegTestPdcp : public Object
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("RetxSegTestPdcp").SetParent<Object>().AddConstructor<RetxSegTestPdcp>();
        return tid;
    }

    void Send(Ptr<NrRlcAm> rlc, const std::string& data)
    {
        NrRlcSapProvider::TransmitPdcpPduParameters p;
        p.rnti = 1111;
        p.lcid = 222;
        p.pdcpPdu = Create<Packet>((uint8_t*)data.data(), data.size());
        rlc->GetNrRlcSapProvider()->TransmitPdcpPdu(p);
    }

    void ReceivePdcpPdu(Ptr<Packet> p)
    {
        uint32_t n = p->GetSize();
        std::vector<uint8_t> buf(n);
        p->CopyData(buf.data(), n);
        m_rx.append((char*)buf.data(), n);
        m_rxCount++;
    }

    std::string m_rx;
    uint32_t m_rxCount{0};
};

class RetxSegTestPdcpSapUser : public NrRlcSapUser
{
  public:
    RetxSegTestPdcpSapUser(RetxSegTestPdcp* o)
        : m_o(o)
    {
    }

    void ReceivePdcpPdu(Ptr<Packet> p) override
    {
        m_o->ReceivePdcpPdu(p);
    }

    RetxSegTestPdcp* m_o;
};

/**
 * Minimal MAC SAP provider: forwards PDUs to the peer RLC with headers
 * intact (perfect channel). Optionally drops the first whole (unsegmented)
 * DATA PDU so the receiver can only rebuild the SDU from segments.
 */
class RetxSegTestMac : public Object, public NrMacSapProvider
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("RetxSegTestMac").SetParent<Object>().AddConstructor<RetxSegTestMac>();
        return tid;
    }

    void SetPeerRlc(Ptr<NrRlcAm> peer)
    {
        m_peerRlc = peer;
    }

    void SetRlc(Ptr<NrRlcAm> rlc)
    {
        m_rlc = rlc;
    }

    void SetDropFirstWhole(bool drop)
    {
        m_dropFirstWhole = drop;
    }

    void TxOpp(uint32_t bytes)
    {
        NrMacSapUser::TxOpportunityParameters t;
        t.bytes = bytes;
        t.layer = 0;
        t.harqId = 0;
        t.componentCarrierId = 0;
        t.rnti = 1111;
        t.lcid = 222;
        m_rlc->GetNrMacSapUser()->NotifyTxOpportunity(t);
    }

    void TransmitPdu(TransmitPduParameters params) override
    {
        m_txPdus++;
        NrRlcAmHeader h;
        params.pdu->PeekHeader(h);
        if (m_dropFirstWhole && h.IsDataPdu() &&
            h.GetResegmentationFlag() == NrRlcAmHeader::PDU)
        {
            m_dropFirstWhole = false;
            return;
        }
        if (h.IsDataPdu() && h.GetResegmentationFlag() == NrRlcAmHeader::SEGMENT)
        {
            m_txSegments++;
        }
        NrMacSapUser::ReceivePduParameters rx(params.pdu, params.rnti, params.lcid);
        Simulator::Schedule(Seconds(0.001),
                            &NrMacSapUser::ReceivePdu,
                            m_peerRlc->GetNrMacSapUser(),
                            rx);
    }

    void ReportBufferStatus(ReportBufferStatusParameters) override
    {
    }

    Ptr<NrRlcAm> m_peerRlc;
    Ptr<NrRlcAm> m_rlc;
    bool m_dropFirstWhole{false};
    uint32_t m_txPdus{0};
    uint32_t m_txSegments{0};
};

/**
 * Inject a STATUS PDU NACKing SN 0 (ACK_SN 1) into the TX RLC.
 */
void
InjectNack(Ptr<NrRlcAm> txRlc)
{
    Ptr<Packet> p = Create<Packet>();
    NrRlcAmHeader st;
    st.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
    st.PushNack(0);
    st.SetAckSn(nr::SequenceNumber10(1));
    p->AddHeader(st);
    // Real MAC arrivals always carry a tag; attach one for DoReceivePdu.
    NrRlcTag tag(Simulator::Now());
    p->AddByteTag(tag, 1, 4);
    NrMacSapUser::ReceivePduParameters rx(p, 1111, 222);
    txRlc->GetNrMacSapUser()->ReceivePdu(rx);
}

/**
 * Inject a STATUS PDU ACKing everything up to ACK_SN 1 (no NACKs).
 */
void
InjectAck(Ptr<NrRlcAm> txRlc)
{
    Ptr<Packet> p = Create<Packet>();
    NrRlcAmHeader st;
    st.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
    st.SetAckSn(nr::SequenceNumber10(1));
    p->AddHeader(st);
    NrRlcTag tag(Simulator::Now());
    p->AddByteTag(tag, 1, 4);
    NrMacSapUser::ReceivePduParameters rx(p, 1111, 222);
    txRlc->GetNrMacSapUser()->ReceivePdu(rx);
}

} // namespace

/**
 * Retransmit a NACKed 100 B PDU through 20 B grants: the TX side must emit
 * segments and the RX side must reassemble exactly the original SDU (the
 * original whole PDU is dropped, so only segments can rebuild it).
 */
class NrRlcAmRetxSegReassembleTestCase : public TestCase
{
  public:
    NrRlcAmRetxSegReassembleTestCase()
        : TestCase("AM retx re-segmentation reassembles the original SDU")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NrRlcAm> txRlc = CreateObject<NrRlcAm>();
        txRlc->SetRnti(1111);
        txRlc->SetLcId(222);
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        rxRlc->SetRnti(1111);
        rxRlc->SetLcId(222);
        Ptr<RetxSegTestPdcp> txPdcp = CreateObject<RetxSegTestPdcp>();
        Ptr<RetxSegTestPdcp> rxPdcp = CreateObject<RetxSegTestPdcp>();
        RetxSegTestPdcpSapUser txUser(GetPointer(txPdcp)), rxUser(GetPointer(rxPdcp));
        txRlc->SetNrRlcSapUser(&txUser);
        rxRlc->SetNrRlcSapUser(&rxUser);
        Ptr<RetxSegTestMac> txMac = CreateObject<RetxSegTestMac>();
        Ptr<RetxSegTestMac> rxMac = CreateObject<RetxSegTestMac>();
        txMac->SetRlc(txRlc);
        rxMac->SetRlc(rxRlc);
        txMac->SetPeerRlc(rxRlc);
        rxMac->SetPeerRlc(txRlc);
        txMac->SetDropFirstWhole(true);
        txRlc->SetNrMacSapProvider(GetPointer(txMac));
        rxRlc->SetNrMacSapProvider(GetPointer(rxMac));

        std::string sdu(100, 'A');
        for (size_t i = 0; i < sdu.size(); ++i)
        {
            sdu[i] = 'A' + (i % 26);
        }

        Simulator::Schedule(Seconds(0.10), &RetxSegTestPdcp::Send, txPdcp, txRlc, sdu);
        Simulator::Schedule(Seconds(0.15), &RetxSegTestMac::TxOpp, txMac, 200);
        Simulator::Schedule(Seconds(0.30), &InjectNack, txRlc);
        // Shrinking grants force repeated re-segmentation of the 104 B PDU.
        Simulator::Schedule(Seconds(0.35), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.40), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.45), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.50), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.55), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.60), &RetxSegTestMac::TxOpp, txMac, 200);
        Simulator::Schedule(Seconds(0.70), &InjectAck, txRlc);

        Simulator::Stop(Seconds(1.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rxPdcp->m_rx,
                              sdu,
                              "RX did not reassemble the original SDU from segments");
        NS_TEST_ASSERT_MSG_EQ(rxPdcp->m_rxCount,
                              1,
                              "RX delivered the SDU more than once");
        NS_TEST_ASSERT_MSG_GT(txMac->m_txSegments,
                              2,
                              "TX did not emit re-segmented retx PDUs");

        Simulator::Destroy();
    }
};

/**
 * A header+1 grant mid-sequence must not assert or stall: the remainder waits
 * for the next grant and reassembly still completes.
 */
class NrRlcAmRetxSegTinyGrantTestCase : public TestCase
{
  public:
    NrRlcAmRetxSegTinyGrantTestCase()
        : TestCase("AM retx re-segmentation survives a header+1 grant")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NrRlcAm> txRlc = CreateObject<NrRlcAm>();
        txRlc->SetRnti(1111);
        txRlc->SetLcId(222);
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        rxRlc->SetRnti(1111);
        rxRlc->SetLcId(222);
        Ptr<RetxSegTestPdcp> txPdcp = CreateObject<RetxSegTestPdcp>();
        Ptr<RetxSegTestPdcp> rxPdcp = CreateObject<RetxSegTestPdcp>();
        RetxSegTestPdcpSapUser txUser(GetPointer(txPdcp)), rxUser(GetPointer(rxPdcp));
        txRlc->SetNrRlcSapUser(&txUser);
        rxRlc->SetNrRlcSapUser(&rxUser);
        Ptr<RetxSegTestMac> txMac = CreateObject<RetxSegTestMac>();
        Ptr<RetxSegTestMac> rxMac = CreateObject<RetxSegTestMac>();
        txMac->SetRlc(txRlc);
        rxMac->SetRlc(rxRlc);
        txMac->SetPeerRlc(rxRlc);
        rxMac->SetPeerRlc(txRlc);
        txMac->SetDropFirstWhole(true);
        txRlc->SetNrMacSapProvider(GetPointer(txMac));
        rxRlc->SetNrMacSapProvider(GetPointer(rxMac));

        std::string sdu(100, 'B');
        for (size_t i = 0; i < sdu.size(); ++i)
        {
            sdu[i] = 'B' + (i % 26);
        }

        Simulator::Schedule(Seconds(0.10), &RetxSegTestPdcp::Send, txPdcp, txRlc, sdu);
        Simulator::Schedule(Seconds(0.15), &RetxSegTestMac::TxOpp, txMac, 200);
        Simulator::Schedule(Seconds(0.30), &InjectNack, txRlc);
        Simulator::Schedule(Seconds(0.35), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.40), &RetxSegTestMac::TxOpp, txMac, 20);
        // Header (4 B) + 1 B: a 1-byte tail cannot be split further, the TX
        // must wait for the next grant instead of asserting.
        Simulator::Schedule(Seconds(0.45), &RetxSegTestMac::TxOpp, txMac, 5);
        Simulator::Schedule(Seconds(0.50), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.55), &RetxSegTestMac::TxOpp, txMac, 20);
        Simulator::Schedule(Seconds(0.60), &RetxSegTestMac::TxOpp, txMac, 200);
        Simulator::Schedule(Seconds(0.70), &InjectAck, txRlc);

        Simulator::Stop(Seconds(1.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rxPdcp->m_rx,
                              sdu,
                              "RX did not reassemble the SDU after a header+1 grant");

        Simulator::Destroy();
    }
};

class NrRlcAmRetxSegTestSuite : public TestSuite
{
  public:
    NrRlcAmRetxSegTestSuite()
        : TestSuite("nr-rlc-am-retx-seg", Type::UNIT)
    {
        AddTestCase(new NrRlcAmRetxSegReassembleTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NrRlcAmRetxSegTinyGrantTestCase, TestCase::Duration::QUICK);
    }
};

static NrRlcAmRetxSegTestSuite g_nrRlcAmRetxSegTestSuite;
