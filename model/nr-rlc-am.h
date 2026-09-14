// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_RLC_AM_H
#define NR_RLC_AM_H

#include "nr-rlc-sequence-number.h"
#include "nr-rlc.h"

#include "ns3/event-id.h"
#include "ns3/nr-export.h"

#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace ns3
{

/**
 * LTE RLC Acknowledged Mode (AM), see 3GPP TS 36.322
 */
class NR_EXPORT NrRlcAm : public NrRlc
{
  public:
    NrRlcAm();
    ~NrRlcAm() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * RLC SAP
     *
     * @param p packet
     */
    void DoTransmitPdcpPdu(Ptr<Packet> p) override;

    /**
     * MAC SAP
     *
     * @param txOpParams the NrMacSapUser::TxOpportunityParameters
     */
    void DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams) override;
    /**
     * Notify HARQ delivery failure
     */
    void DoNotifyHarqDeliveryFailure() override;
    void DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams) override;

  private:
    /**
     * This method will schedule a timeout at WaitReplyTimeout interval
     * in the future, unless a timer is already running for the cache,
     * in which case this method does nothing.
     */
    void ExpireReorderingTimer();
    /// Expire poll retransmitter
    void ExpirePollRetransmitTimer();
    /// Expire BSR timer
    void ExpireBsrTimer();

    /**
     * method called when the T_status_prohibit timer expires
     *
     */
    void ExpireStatusProhibitTimer();

    /**
     * method called when the T_status_prohibit timer expires
     *
     * @param seqNumber nr::SequenceNumber10
     * @returns true is inside receiving window
     */
    bool IsInsideReceivingWindow(nr::SequenceNumber10 seqNumber);
    //
    //   void ReassembleOutsideWindow ();
    //   void ReassembleSnLessThan (uint16_t seqNumber);
    //

    void ReestablishRxSide() override;

    /**
     * @brief Build and transmit one AMD PDU segment for a retransmission (TS 36.322 5.2.1).
     *
     * Emits the next untransmitted byte range of the original AMD PDU held in
     * m_retxBuffer[seqNumberValue] (RF = 1, SO/LSF set, single Data field
     * element, no E/LI extension), sized to fit txOpportunityBytes, and advances
     * m_nextSegmentOffset[seqNumberValue]. The last segment moves the original
     * PDU back to the txed buffer to await acknowledgement.
     *
     * @param seqNumberValue SN of the AMD PDU to re-segment
     * @param txOpportunityBytes grant size in bytes
     * @param txOpParams the grant (layer/HARQ/CC passed through to the MAC)
     * @return size of the emitted segment, or 0 if the whole PDU fits (caller
     *         retransmits it as is) or no useful segment fits the grant
     */
    uint32_t BuildRetxSegment(uint16_t seqNumberValue,
                              uint32_t txOpportunityBytes,
                              const NrMacSapUser::TxOpportunityParameters& txOpParams);

    /**
     * Reassemble and deliver
     *
     * @param packet the packet
     */
    void ReassembleAndDeliver(Ptr<Packet> packet);

    /**
     * Buffer status report
     */
    void DoTransmitBufferStatusReport();

  private:
    /// Grant the STATUS PDU robustness test access to private transmit window state
    friend class NrRlcAmStaleStatusTestCase;
    /// Grant the reassembly resynchronisation regression test access to private reassembly state
    friend class NrRlcAmReassemblyResyncTestCase;
    /// Grant the resegmentation test access to private resegmentation state
    friend class NrRlcAmResegmentationTestCase;

    /**
     * @brief Store an incoming (from layer above us) PDU, waiting to transmit it
     */
    struct NR_EXPORT TxPdu
    {
        /**
         * @brief TxPdu default constructor
         * @param pdu the PDU
         * @param time the arrival time
         */
        TxPdu(const Ptr<Packet>& pdu, const Time& time)
            : m_pdu(pdu),
              m_waitingSince(time)
        {
        }

        TxPdu() = delete;

        Ptr<Packet> m_pdu;   ///< PDU
        Time m_waitingSince; ///< Layer arrival time
    };

    std::vector<TxPdu> m_txonBuffer; ///< Transmission buffer

    /// RetxPdu structure
    struct NR_EXPORT RetxPdu
    {
        Ptr<Packet> m_pdu;    ///< PDU
        uint16_t m_retxCount; ///< retransmit count
        Time m_waitingSince;  ///< Layer arrival time
    };

    std::vector<RetxPdu> m_txedBuffer; ///< Buffer for transmitted and retransmitted PDUs
                                       ///< that have not been acked but are not considered
                                       ///< for retransmission
    std::vector<RetxPdu> m_retxBuffer; ///< Buffer for PDUs considered for retransmission

    /**
     * @brief Next Segment Offset (SO) to use when re-segmenting the AMD PDU
     *        whose SN is the map key (TS 36.322 6.2.2.7).
     *
     * The original AMD PDU stays in m_retxBuffer; each emitted AMD PDU segment
     * advances the offset by the number of original Data-field bytes it carried,
     * so successive segments cover successive byte ranges of the original PDU.
     * Entries are erased when the SN is acknowledged or requeued.
     */
    std::map<uint16_t, uint16_t> m_nextSegmentOffset;

    uint32_t m_maxTxBufferSize; ///< maximum transmission buffer size
    uint32_t m_txonBufferSize;  ///< transmit on buffer size
    uint32_t m_retxBufferSize;  ///< retransmit buffer size
    uint32_t m_txedBufferSize;  ///< transmit ed buffer size

    bool m_statusPduRequested;      ///< status PDU requested
    uint32_t m_statusPduBufferSize; ///< status PDU buffer size

    /// PduBuffer structure
    struct NR_EXPORT PduBuffer
    {
        nr::SequenceNumber10 m_seqNumber;      ///< sequence number
        std::list<Ptr<Packet>> m_byteSegments; ///< byte segments

        bool m_pduComplete; ///< PDU complete?
    };

    std::map<uint16_t, PduBuffer> m_rxonBuffer; ///< Reception buffer

    /**
     * @brief Byte ranges (offsets into the original AMD PDU Data field) received
     *        for the SN that is the map key (TS 36.322 5.1.3.2.2).
     *
     * One interval per received AMD PDU segment; whole AMD PDUs (RF = 0) cover
     * [0, dataSize). The SN's PDU is complete when the union of its intervals
     * covers [0, m_rxPduDataSize[sn]) contiguously, i.e. IsPduComplete(sn).
     * Entries are erased when the SN leaves the reception buffer. An entry
     * without a data-size starts uncovered (nothing is assumed received).
     */
    std::map<uint16_t, std::set<std::pair<uint16_t, uint16_t>>> m_rxSegmentRanges;
    /// Data-field size of the original AMD PDU per SN, known once a segment
    /// with LSF = 1 (or a whole AMD PDU) is received.
    std::map<uint16_t, uint16_t> m_rxPduDataSize;

    /**
     * @brief Whether every byte of the AMD PDU with the given SN has been received.
     *
     * The union of the received intervals must cover [0, m_rxPduDataSize[sn])
     * contiguously: intervals are walked in increasing offset order from 0 and
     * each must start at or before the running coverage end.
     *
     * @param seqNumberValue the SN value
     * @return true if all bytes are received, false otherwise (including when
     *         the total size is still unknown or nothing was received yet)
     */
    bool IsPduComplete(uint16_t seqNumberValue) const;

    // SDU reassembly
    std::list<Ptr<Packet>> m_sdusBuffer; ///< List of SDUs in a packet (PDU)

    /**
     * State variables. See section 7.1 in TS 36.322
     */
    // Transmitting side
    nr::SequenceNumber10 m_vtA;    ///< VT(A)
    nr::SequenceNumber10 m_vtMs;   ///< VT(MS)
    nr::SequenceNumber10 m_vtS;    ///< VT(S)
    nr::SequenceNumber10 m_pollSn; ///< POLL_SN

    // Receiving side
    nr::SequenceNumber10 m_vrR;  ///< VR(R)
    nr::SequenceNumber10 m_vrMr; ///< VR(MR)
    nr::SequenceNumber10 m_vrX;  ///< VR(X)
    nr::SequenceNumber10 m_vrMs; ///< VR(MS)
    nr::SequenceNumber10 m_vrH;  ///< VR(H)

    /**
     * Counters. See section 7.1 in TS 36.322
     */
    uint32_t m_pduWithoutPoll;  ///< PDU without poll
    uint32_t m_byteWithoutPoll; ///< byte without poll

    /**
     * Constants. See section 7.2 in TS 36.322
     */
    uint16_t m_windowSize;

    /**
     * Timers. See section 7.3 in TS 36.322
     */
    EventId m_pollRetransmitTimer;   ///< poll retransmit timer
    Time m_pollRetransmitTimerValue; ///< poll retransmit time value
    EventId m_reorderingTimer;       ///< reordering timer
    Time m_reorderingTimerValue;     ///< reordering timer value
    EventId m_statusProhibitTimer;   ///< status prohibit timer
    Time m_statusProhibitTimerValue; ///< status prohibit timer value
    EventId m_bsrTimer;              ///< BSR timer
    Time m_bsrTimerValue;            ///< BSR timer value

    /**
     * Configurable parameters. See section 7.4 in TS 36.322
     */
    uint16_t m_maxRetxThreshold; ///< \todo How these parameters are configured???
    uint16_t m_pollPdu;          ///< poll PDU
    uint16_t m_pollByte;         ///< poll byte

    /// Set once the maxRetxThreshold has been reached and the RLF callback fired,
    /// so the radio-link-failure indication is raised at most once per RLC entity.
    bool m_maxRetxReachedNotified{false};

    bool m_txOpportunityForRetxAlwaysBigEnough; ///< transmit opportunity for retransmit?
    bool m_pollRetransmitTimerJustExpired;      ///< poll retransmit timer just expired?

    /**
     * SDU Reassembling state
     */
    enum ReassemblingState_t
    {
        NONE = 0,
        WAITING_S0_FULL = 1,
        WAITING_SI_SF = 2
    };

    ReassemblingState_t m_reassemblingState; ///< reassembling state
    Ptr<Packet> m_keepS0;                    ///< keep S0

    /**
     * Expected Sequence Number
     */
    nr::SequenceNumber10 m_expectedSeqNumber;

    /**
     * @brief Reassemble a complete AMD PDU from its received segments and deliver it.
     *
     * Concatenates the Data fields of the buffered segments in SO order into
     * the original AMD PDU payload and passes it to ReassembleAndDeliver, so
     * segmented retransmissions deliver exactly like whole PDUs. A lone whole
     * PDU is delivered directly.
     *
     * @param pduBuffer the reception-buffer entry holding the segments
     */
    void ReassembleCompletePdu(PduBuffer& pduBuffer);
};

} // namespace ns3

#endif // NR_RLC_AM_H
