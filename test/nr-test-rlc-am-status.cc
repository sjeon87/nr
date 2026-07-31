// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rlc-am-status.cc
 *
 * @brief Test suite `nr-test-rlc-am-status`: white-box regression tests of the NrRlcAm
 * robustness against stale, duplicated or reordered inputs. STATUS PDUs have no sequence
 * protection: HARQ processes can deliver them out of order, and after a handover both RLC
 * entities are re-created, so old-epoch PDUs can reach a fresh entity at an aliased 10-bit
 * SN. The cases verify that a NACK for an SN that was already positively acknowledged and
 * released is treated as acknowledged (instead of asserting or freezing VT(A)); that a
 * STATUS PDU whose ACK_SN falls outside the transmit window is discarded without touching
 * the buffers; that a valid STATUS PDU still moves NACKed SNs to the retransmission buffer
 * and advances VT(A); and that a data PDU whose framing info contradicts the reassembly
 * state resynchronises the reassembler (discarding the stale partial SDU) instead of
 * leaving segments queued in m_sdusBuffer where they would corrupt every subsequently
 * delivered SDU. A data-integrity case additionally drives a transmitting entity
 * through segmentation and concatenation and verifies byte-exactly that the receiving
 * entity delivers the transmitted SDUs in order, under lossless, reordered and
 * ARQ-retransmission conditions.
 */

#include "nr-rlc-test-utils.h"
#include "nr-test-entities.h"

#include "ns3/log.h"
#include "ns3/nr-rlc-am-header.h"
#include "ns3/nr-rlc-am.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

namespace
{

/**
 * Build an AMD PDU carrying the given data-field segments, with an RLC tag covering
 * its header.
 *
 * @param sn the RLC sequence number
 * @param framingInfo the FI field
 * @param segments payload segments (fill byte and size per segment)
 * @param requestPoll whether to set the polling bit
 * @return the AMD PDU
 */
Ptr<Packet>
MakeAmdPdu(uint16_t sn,
           uint8_t framingInfo,
           const std::vector<NrRlcSduSpec>& segments,
           bool requestPoll)
{
    Ptr<Packet> pdu = Create<Packet>();
    for (const auto& [fill, size] : segments)
    {
        Ptr<Packet> segment = MakeUniformSdu(fill, size);
        if (pdu->GetSize() > 0)
        {
            pdu->AddAtEnd(segment);
        }
        else
        {
            pdu = segment;
        }
    }

    NrRlcAmHeader header;
    header.SetDataPdu();
    header.SetSequenceNumber(nr::SequenceNumber10(sn));
    header.SetResegmentationFlag(NrRlcAmHeader::PDU);
    header.SetLastSegmentFlag(NrRlcAmHeader::LAST_PDU_SEGMENT);
    header.SetSegmentOffset(0);
    header.SetFramingInfo(framingInfo);
    header.SetPollingBit(requestPoll ? NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED
                                     : NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);
    for (uint32_t i = 0; i + 1 < segments.size(); i++)
    {
        header.PushExtensionBit(NrRlcAmHeader::E_LI_FIELDS_FOLLOWS);
        header.PushLengthIndicator(segments[i].second);
    }
    header.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);
    pdu->AddHeader(header);

    NrRlcTag rlcTag(Simulator::Now());
    pdu->AddByteTag(rlcTag, 1, header.GetSerializedSize());

    return pdu;
}

/**
 * Deliver one full-SDU AMD PDU to the RLC AM entity through the MAC SAP.
 *
 * @param rlc the receiving RLC entity
 * @param sn the RLC sequence number
 * @param size the SDU payload size in bytes
 * @param requestPoll whether to set the polling bit
 */
void
DeliverAmDataPdu(Ptr<NrRlcAm> rlc, uint16_t sn, uint32_t size, bool requestPoll)
{
    const uint8_t fi00 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;
    DeliverPduToRlc(rlc, MakeAmdPdu(sn, fi00, {{0, size}}, requestPoll));
}

/**
 * Deliver a STATUS PDU to the RLC AM entity through the MAC SAP.
 *
 * @param rlc the RLC entity under test
 * @param ackSn the ACK_SN value
 * @param nacks the NACK_SN values
 */
void
DeliverStatusPdu(Ptr<NrRlcAm> rlc, uint16_t ackSn, const std::vector<int>& nacks)
{
    NrRlcAmHeader header;
    header.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
    header.SetAckSn(nr::SequenceNumber10(ackSn));
    for (int nack : nacks)
    {
        header.PushNack(nack);
    }

    Ptr<Packet> status = Create<Packet>();
    status->AddHeader(header);

    NrRlcTag rlcTag(Simulator::Now());
    status->AddByteTag(rlcTag, 1, header.GetSerializedSize());

    DeliverPduToRlc(rlc, status);
}

} // namespace

/**
 * @ingroup tests
 *
 * @brief Regression test for NrRlcAm STATUS PDU robustness.
 *
 * Drives a transmitting AM entity to emit SNs 0..2, then feeds it STATUS PDUs that
 * are reordered or stale and verifies the entity neither aborts nor corrupts its
 * transmit window bookkeeping.
 */
class NrRlcAmStaleStatusTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmStaleStatusTestCase()
        : NrRlcTestCaseBase("Test RLC AM TX: stale and reordered STATUS PDUs are handled safely")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmStaleStatusTestCase::DoRun()
{
    Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::AM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());

    // Emit three AMD PDUs (SN=0, 1, 2): interleave SDU arrivals and transmission
    // opportunities so each opportunity carries exactly one SDU.
    TransmitSdusAligned(rlc, {{'A', 20}, {'B', 20}, {'C', 20}}, 100);

    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 0, "precondition: VT(A) = 0");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtS.GetValue(), 3, "precondition: VT(S) = 3");
    for (uint16_t sn = 0; sn < 3; sn++)
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(sn).m_pdu != nullptr),
                              true,
                              "precondition: SN=" << sn << " must be in the txed buffer");
    }

    // A stale STATUS PDU (old epoch or corrupted) whose ACK_SN falls outside the
    // transmit window [VT(A), VT(S)] must be discarded without touching anything.
    DeliverStatusPdu(rlc, 700, {});
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(),
                          0,
                          "an out-of-window ACK_SN must not advance VT(A)");
    for (uint16_t sn = 0; sn < 3; sn++)
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(sn).m_pdu != nullptr),
                              true,
                              "an out-of-window ACK_SN must not release SN=" << sn);
    }

    // Reordering scenario: the NEWER STATUS PDU (ACK_SN=3, NACK 0) arrives first.
    // SN=0 moves to the retransmission buffer; SN=1 and SN=2 are positively
    // acknowledged and released; VT(A) stays at the NACKed SN=0.
    DeliverStatusPdu(rlc, 3, {0});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(0).m_pdu != nullptr),
                          true,
                          "the NACKed SN=0 must move to the retx buffer");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(1).m_pdu == nullptr),
                          true,
                          "the ACKed SN=1 must be released from the txed buffer");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(2).m_pdu == nullptr),
                          true,
                          "the ACKed SN=2 must be released from the txed buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 0, "VT(A) must hold at the NACKed SN");

    // The OLDER STATUS PDU (ACK_SN=3, NACK 0, NACK 1) arrives late: its NACK for
    // SN=1 refers to a PDU already acknowledged and released. Before the fix this
    // hit NS_ASSERT(m_retxBuffer.at(sn).m_pdu) and aborted the simulation; it must
    // instead be treated as acknowledged.
    DeliverStatusPdu(rlc, 3, {0, 1});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(1).m_pdu == nullptr),
                          true,
                          "a stale NACK for a released SN must not resurrect it");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(),
                          0,
                          "VT(A) must still hold at the genuinely NACKed SN=0");

    // A subsequent valid STATUS PDU acknowledging everything releases the retx
    // PDU and advances VT(A) to VT(S): normal operation is unaffected.
    DeliverStatusPdu(rlc, 3, {});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(0).m_pdu == nullptr),
                          true,
                          "the final ACK must release the retx PDU");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 3, "the final ACK must advance VT(A) to VT(S)");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txedBufferSize, 0, "txed buffer size must drain to zero");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_retxBufferSize, 0, "retx buffer size must drain to zero");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the NrRlcAm reassembly resynchronisation.
 *
 * A data PDU delivered in sequence (SN == expected SN, e.g. an old-epoch PDU
 * aliasing into a fresh entity after handover) whose framing info contradicts the
 * reassembly state used to fall through the "Transition not possible" defaults
 * WITHOUT consuming m_sdusBuffer: the stale segments stayed queued and corrupted
 * every subsequently delivered SDU (silent PDCP/RRC corruption). The reassembler
 * must instead resynchronise on an SDU boundary, discard un-completable partial
 * SDUs, salvage complete SDUs, and deliver later PDUs byte-exact.
 */
class NrRlcAmReassemblyResyncTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmReassemblyResyncTestCase()
        : NrRlcTestCaseBase("Test RLC AM RX: FI/state contradiction resynchronises the reassembler")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmReassemblyResyncTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;
    const uint8_t fi01 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::NO_LAST_BYTE;
    const uint8_t fi10 = NrRlcAmHeader::NO_FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;

    // Feed a PDU straight to the reassembler, bypassing the reception window.
    auto reassemble = [](Ptr<NrRlcAm> rlc,
                         uint16_t sn,
                         uint8_t framingInfo,
                         const std::vector<NrRlcSduSpec>& segments) {
        rlc->ReassembleAndDeliver(MakeAmdPdu(sn, framingInfo, segments, false));
    };

    // Case 1: mid-SDU (WAITING_SI_SF, S0 held) and an in-sequence PDU that STARTS a
    // new SDU (old-epoch alias). The stale head must be discarded and the new SDU
    // delivered intact -- never concatenated with the held head.
    {
        Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
        NrRlcTestSduSink sink;
        rlc->SetNrRlcSapUser(sink.GetSapUser());

        reassemble(rlc, 0, fi01, {{'S', 40}}); // head of an SDU, continuation never arrives
        NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                              (int)NrRlcAm::WAITING_SI_SF,
                              "precondition: reassembler holds a partial SDU");

        reassemble(rlc, 1, fi00, {{'A', 21}}); // in-sequence, but starts a new SDU

        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "exactly one SDU must be delivered");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(0), 'A', 21),
                              true,
                              "the new SDU must be delivered intact, not concatenated with the "
                              "stale head");
        NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                              (int)NrRlcAm::WAITING_S0_FULL,
                              "the reassembler must resynchronise to an SDU boundary");
        NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 == nullptr),
                              true,
                              "the stale partial SDU must be discarded");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_sdusBuffer.size(),
                              0,
                              "no segments may remain queued in m_sdusBuffer");
    }

    // Case 2: at an SDU boundary (WAITING_S0_FULL) and an in-sequence PDU that
    // CONTINUES a previous SDU (orphan continuation, e.g. old-epoch tail). Before
    // the fix this fell through without consuming m_sdusBuffer, so the next
    // delivered SDU was corrupted by the stale queued segments. The orphan leading
    // segment must be discarded, the complete trailing SDU salvaged, and the next
    // PDU delivered byte-exact.
    {
        Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
        NrRlcTestSduSink sink;
        rlc->SetNrRlcSapUser(sink.GetSapUser());

        reassemble(rlc, 0, fi00, {{'A', 21}});            // normal full SDU
        reassemble(rlc, 1, fi10, {{'X', 22}, {'B', 23}}); // orphan tail 'X' + full SDU 'B'
        reassemble(rlc, 2, fi00, {{'C', 24}});            // normal full SDU, must arrive clean

        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 3, "exactly three SDUs must be delivered");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(0), 'A', 21),
                              true,
                              "the first SDU must be delivered intact");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(1), 'B', 23),
                              true,
                              "the complete SDU after the orphan tail must be salvaged intact");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(2), 'C', 24),
                              true,
                              "the SDU following the contradiction must not be corrupted by "
                              "stale queued segments");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_sdusBuffer.size(),
                              0,
                              "no segments may remain queued in m_sdusBuffer");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief End-to-end data-integrity test of the RLC AM transmit and receive entities.
 *
 * A transmitting NrRlcAm segments and concatenates SDUs with distinct, identifiable
 * payloads into AMD PDUs (small transmission opportunities force both). The captured
 * PDUs are delivered to fresh receiving entities under three conditions -- lossless
 * in order, with two PDUs swapped (HARQ reordering), and with one PDU withheld and
 * delivered last (an ARQ retransmission closing the gap) -- verifying byte-exactly
 * that every SDU is delivered equal to what was transmitted, in order, and that
 * nothing is delivered past an open gap (AM delivers strictly in sequence).
 */
class NrRlcAmDataIntegrityTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmDataIntegrityTestCase()
        : NrRlcTestCaseBase("Test RLC AM: transmitted data is received byte-exact under "
                            "segmentation, reordering and retransmission")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmDataIntegrityTestCase::DoRun()
{
    const std::vector<NrRlcSduSpec> txSdus =
        {{'A', 45}, {'B', 52}, {'C', 59}, {'D', 66}, {'E', 73}, {'F', 80}};

    // Transmit side: queue the SDUs and grant small opportunities so the AMD PDUs
    // both segment large SDUs and concatenate small remainders.
    Ptr<NrRlcAm> txRlc = CreateObject<NrRlcAm>();
    NrRlcTestCaptureMac txCapture;
    txRlc->SetNrMacSapProvider(&txCapture);
    QueueSdus(txRlc, txSdus);
    GrantTxOpportunities(txRlc, 44, 20);
    const std::vector<Ptr<Packet>>& pdus = txCapture.m_pdus;

    CheckSegmentationAndConcatenation<NrRlcAmHeader>(pdus);

    // Scenario 1: all PDUs delivered in order -- every SDU arrives byte-exact.
    {
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        VerifyDelivered(sink, txSdus, "lossless in-order");
    }

    // Scenario 2: two adjacent PDUs swapped (HARQ processes deliver out of order) --
    // the receiving window restores SN order and every SDU arrives byte-exact.
    {
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        std::vector<Ptr<Packet>> reordered = pdus;
        std::swap(reordered.at(2), reordered.at(3));
        for (const auto& pdu : reordered)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        VerifyDelivered(sink, txSdus, "reordered");
    }

    // Scenario 3: one mid-stream PDU is withheld and delivered last, like an ARQ
    // retransmission closing the gap. AM delivers strictly in sequence: while the
    // gap is open, no SDU at or beyond it may be delivered (and never partially);
    // once the retransmission arrives, everything must be delivered in order and
    // byte-exact.
    {
        const uint32_t retxIndex = pdus.size() / 2;

        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        for (uint32_t i = 0; i < pdus.size(); i++)
        {
            if (i == retxIndex)
            {
                continue;
            }
            DeliverPduToRlc(rxRlc, pdus.at(i));
        }

        // With the gap open, every delivered SDU must still be byte-exact (a prefix
        // of the transmitted sequence: nothing at or beyond the gap).
        NS_TEST_ASSERT_MSG_EQ((sink.m_sdus.size() < txSdus.size()),
                              true,
                              "with an open gap, not all SDUs may be delivered");
        for (uint32_t i = 0; i < sink.m_sdus.size(); i++)
        {
            NS_TEST_ASSERT_MSG_EQ(
                IsUniformSdu(sink.m_sdus.at(i), txSdus[i].first, txSdus[i].second),
                true,
                "SDUs delivered before the gap must be byte-exact and in order");
        }

        // The "retransmission" closes the gap: everything is delivered in order.
        DeliverPduToRlc(rxRlc, pdus.at(retxIndex));
        VerifyDelivered(sink, txSdus, "retransmission closes the gap");
    }

    // Scenario 4: SN epoch mixing after a handover, with real transmitters on both
    // sides. An OLD-epoch transmitter emits three SDU-aligned PDUs; its SN=2 PDU is
    // still in flight when the entities are re-created and reaches the fresh
    // receiver first, occupying the SN=2 slot. The NEW-epoch transmitter restarts
    // at SN=0: the stale PDU is delivered at the aliased position (the
    // resynchronisation discards the partial new-epoch SDU it interrupts, then the
    // orphan continuation), and the genuine new-epoch SN=2 is discarded as below
    // the receiving window. Data integrity demands: every delivered SDU is
    // byte-exact (old- or new-epoch -- never a mix), the stale SDU is delivered
    // exactly once, new-epoch SDUs keep their relative order, and exactly the
    // new-epoch SDUs with data in the shadowed SN=2 PDU are missing.
    {
        const std::vector<NrRlcSduSpec> oldSdus = {{'X', 30}, {'Y', 34}, {'Z', 38}};
        Ptr<NrRlcAm> oldTxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac oldCapture;
        oldTxRlc->SetNrMacSapProvider(&oldCapture);
        TransmitSdusAligned(oldTxRlc, oldSdus, 60);
        NS_TEST_ASSERT_MSG_EQ(oldCapture.m_pdus.size(),
                              3,
                              "epoch-mix sanity: one SDU-aligned PDU per old-epoch SDU");
        Ptr<Packet> stalePdu = oldCapture.m_pdus.at(2); // old-epoch SN=2, carries 'Z'

        // Expected: all new-epoch SDUs except those with data in the new SN=2 PDU.
        std::set<uint8_t> shadowedFills = FillsInPdu<NrRlcAmHeader>(pdus.at(2));
        std::vector<NrRlcSduSpec> expectedNew;
        for (const auto& sdu : txSdus)
        {
            if (shadowedFills.count(sdu.first) == 0)
            {
                expectedNew.push_back(sdu);
            }
        }
        NS_TEST_ASSERT_MSG_EQ((expectedNew.size() < txSdus.size()),
                              true,
                              "epoch-mix sanity: the shadowed PDU must carry SDU data");

        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        DeliverPduToRlc(rxRlc, stalePdu); // in-flight old-epoch PDU arrives first
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }

        // Every delivered SDU must be byte-exact against its epoch's original.
        std::map<uint8_t, uint32_t> allSpecs;
        for (const auto& [fill, size] : txSdus)
        {
            allSpecs[fill] = size;
        }
        for (const auto& [fill, size] : oldSdus)
        {
            allSpecs[fill] = size;
        }
        uint32_t staleDeliveries = 0;
        std::vector<uint8_t> deliveredNewFills;
        for (uint32_t i = 0; i < sink.m_sdus.size(); i++)
        {
            std::vector<uint8_t> buf(sink.m_sdus.at(i)->GetSize());
            sink.m_sdus.at(i)->CopyData(buf.data(), buf.size());
            const uint8_t fill = buf.empty() ? 0 : buf.front();
            NS_TEST_ASSERT_MSG_EQ((allSpecs.count(fill) > 0),
                                  true,
                                  "epoch mix: delivered SDU " << i
                                                              << " must match a transmitted SDU");
            NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(i), fill, allSpecs[fill]),
                                  true,
                                  "epoch mix: delivered SDU "
                                      << i << " ('" << (char)fill
                                      << "') must be byte-exact, never an epoch mix");
            if (fill == 'Z')
            {
                staleDeliveries++;
            }
            else
            {
                deliveredNewFills.push_back(fill);
            }
        }
        NS_TEST_ASSERT_MSG_EQ(staleDeliveries,
                              1,
                              "epoch mix: the stale SDU must be delivered exactly once");
        NS_TEST_ASSERT_MSG_EQ(deliveredNewFills.size(),
                              expectedNew.size(),
                              "epoch mix: exactly the un-shadowed new-epoch SDUs are delivered");
        for (uint32_t i = 0; i < std::min(deliveredNewFills.size(), expectedNew.size()); i++)
        {
            NS_TEST_ASSERT_MSG_EQ((char)deliveredNewFills.at(i),
                                  (char)expectedNew[i].first,
                                  "epoch mix: new-epoch SDUs must keep their relative order");
        }
    }

    Simulator::Destroy();
}

class NrRlcAmStatusTestSuite : public TestSuite
{
  public:
    NrRlcAmStatusTestSuite()
        : TestSuite("nr-test-rlc-am-status", Type::SYSTEM)
    {
        AddTestCase(new NrRlcAmStaleStatusTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmReassemblyResyncTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmDataIntegrityTestCase(), Duration::QUICK);
    }
};

static NrRlcAmStatusTestSuite g_nrRlcAmStatusTestSuite;

} // namespace ns3
