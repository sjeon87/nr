// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>

#ifndef NR_RLC_H
#define NR_RLC_H

#include "nr-mac-sap.h"
#include "nr-rlc-sap.h"

#include "ns3/nr-export.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/simple-ref-count.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/traced-value.h"
#include "ns3/uinteger.h"

#include <list>

namespace ns3
{

// class NrRlcSapProvider;
// class NrRlcSapUser;
//
// class NrMacSapProvider;
// class NrMacSapUser;

/**
 * This abstract base class defines the API to interact with the Radio Link Control
 * (NR_RLC) in LTE, see 3GPP TS 36.322
 *
 */
class NR_EXPORT NrRlc : public Object // SimpleRefCount<NrRlc>
{
    /// allow NrRlcSpecificNrMacSapUser class friend access
    friend class NrRlcSpecificNrMacSapUser;
    /// allow NrRlcSpecificNrRlcSapProvider<NrRlc> class friend access
    friend class NrRlcSpecificNrRlcSapProvider<NrRlc>;

  public:
    NrRlc();
    ~NrRlc() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     *
     *
     * @param rnti
     */
    void SetRnti(uint16_t rnti);

    /**
     *
     *
     * @param lcId
     */
    void SetLcId(uint8_t lcId);

    /**
     * @param packetDelayBudget
     */
    void SetPacketDelayBudgetMs(uint16_t packetDelayBudget);

    /**
     *
     *
     * @param s the RLC SAP user to be used by this NR_RLC
     */
    void SetNrRlcSapUser(NrRlcSapUser* s);

    /**
     *
     *
     * @return the RLC SAP Provider interface offered to the PDCP by this NR_RLC
     */
    NrRlcSapProvider* GetNrRlcSapProvider();

    /**
     *
     *
     * @param s the MAC SAP Provider to be used by this NR_RLC
     */
    void SetNrMacSapProvider(NrMacSapProvider* s);

    /**
     *
     *
     * @return the MAC SAP User interface offered to the MAC by this NR_RLC
     */
    NrMacSapUser* GetNrMacSapUser();

    /**
     * Set a callback to be invoked once when an RLC-AM entity reaches its
     * maxRetxThreshold (i.e. an AM PDU could not be delivered within the
     * configured number of retransmissions). Per TS 38.331 5.3.10.3 this is a
     * radio-link-failure trigger; the RRC wires this to its RLF handler. Only
     * NrRlcAm fires it; for TM/UM RLC it is never invoked.
     *
     * @param cb the callback to invoke on max-retx
     */
    void SetMaxRetxReachedCallback(Callback<void> cb);

    /**
     * TracedCallback signature for NotifyTxOpportunity events.
     *
     * @param [in] rnti C-RNTI scheduled.
     * @param [in] lcid The logical channel id corresponding to
     *             the sending RLC instance.
     * @param [in] bytes The number of bytes to transmit
     */
    typedef void (*NotifyTxTracedCallback)(uint16_t rnti, uint8_t lcid, uint32_t bytes);

    /**
     * TracedCallback signature for
     *
     * @param [in] rnti C-RNTI scheduled.
     * @param [in] lcid The logical channel id corresponding to
     *             the sending RLC instance.
     * @param [in] bytes The packet size.
     * @param [in] delay Delay since sender timestamp, in ns.
     */
    typedef void (*ReceiveTracedCallback)(uint16_t rnti,
                                          uint8_t lcid,
                                          uint32_t bytes,
                                          uint64_t delay);

    /// @todo MRE What is the sense to duplicate all the interfaces here???
    // NB to avoid the use of multiple inheritance

  protected:
    // Interface forwarded by NrRlcSapProvider
    /**
     * Transmit PDCP PDU
     *
     * @param p packet
     */
    virtual void DoTransmitPdcpPdu(Ptr<Packet> p) = 0;

    /**
     * Split the payload of a received RLC PDU into its data fields, as delimited by
     * the length indicators of the (already removed) RLC header.
     *
     * @tparam RlcHeader the RLC header type, NrRlcHeader or NrRlcAmHeader.
     * @param header Header of the PDU, whose E and LI fields are consumed.
     * @param packet Payload of the PDU, fragmented in place.
     * @param dataFields List the data fields are appended to, in order.
     */
    template <typename RlcHeader>
    void SplitDataFields(RlcHeader& header, Ptr<Packet> packet, std::list<Ptr<Packet>>& dataFields);

    /**
     * Reassemble the data fields carried by a received RLC PDU and deliver the
     * complete SDUs to PDCP, in the order they appear in the PDU.
     *
     * The data fields are consumed: a leading field that continues the held first
     * segment is concatenated with it, a trailing field that does not end on an SDU
     * boundary becomes the new held first segment, and everything in between is a
     * complete SDU. Segments that can no longer be reassembled -- a held segment
     * whose continuation was lost, or a continuation whose first segment is gone --
     * are discarded, which resynchronises the entity on the next SDU boundary.
     *
     * @param dataFields Data fields of the PDU, in order; emptied by the call.
     * @param keepS0 Held first segment (may be null); updated in place.
     * @param firstFieldContinuesSdu Whether the first data field continues the SDU
     *        of a previous PDU (FI = 10 or 11).
     * @param lastFieldIsComplete Whether the last data field ends an SDU
     *        (FI = 00 or 10).
     * @param heldSegmentIsUsable Whether the held first segment is still the
     *        immediate predecessor of this PDU, i.e. no loss was detected and the
     *        framing info agrees with the reassembly state.
     * @return Whether a first segment is being held after the call, i.e. whether
     *         the entity is left mid-SDU.
     */
    bool ReassembleSdus(std::list<Ptr<Packet>>& dataFields,
                        Ptr<Packet>& keepS0,
                        bool firstFieldContinuesSdu,
                        bool lastFieldIsComplete,
                        bool heldSegmentIsUsable);

    NrRlcSapUser* m_rlcSapUser;         ///< RLC SAP user
    NrRlcSapProvider* m_rlcSapProvider; ///< RLC SAP provider

    /// Invoked once when an RLC-AM entity reaches maxRetxThreshold (RLF trigger,
    /// TS 38.331 5.3.10.3). Empty unless wired by the RRC. @see SetMaxRetxReachedCallback
    Callback<void> m_maxRetxReachedCallback;

    // Interface forwarded by NrMacSapUser
    /**
     * Notify transmit opportunity
     *
     * @param params NrMacSapUser::TxOpportunityParameters
     */
    virtual void DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters params) = 0;
    /**
     * Notify HARQ delivery failure
     */
    virtual void DoNotifyHarqDeliveryFailure() = 0;
    /**
     * Receive PDU function
     *
     * @param params the NrMacSapUser::ReceivePduParameters
     */
    virtual void DoReceivePdu(NrMacSapUser::ReceivePduParameters params) = 0;

    NrMacSapUser* m_macSapUser;         ///< MAC SAP user
    NrMacSapProvider* m_macSapProvider; ///< MAC SAP provider

    uint16_t m_rnti; ///< RNTI
    uint8_t m_lcid;  ///< LCID
    uint16_t m_packetDelayBudgetMs{
        UINT16_MAX}; //!< the packet delay budget in ms of the corresponding logical channel

    /**
     * Used to inform of a PDU delivery to the MAC SAP provider
     */
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txPdu;
    /**
     * Used to inform of a PDU reception from the MAC SAP user
     */
    TracedCallback<uint16_t, uint8_t, uint32_t, uint64_t> m_rxPdu;
    /**
     * The trace source fired when the RLC drops a packet before
     * transmission.
     */
    TracedCallback<Ptr<const Packet>> m_txDropTrace;
};

/**
 * NR_RLC Saturation Mode (SM): simulation-specific mode used for
 * experiments that do not need to consider the layers above the NR_RLC.
 * The NR_RLC SM, unlike the standard NR_RLC modes, it does not provide
 * data delivery services to upper layers; rather, it just generates a
 * new NR_RLC PDU whenever the MAC notifies a transmission opportunity.
 *
 */
class NR_EXPORT NrRlcSm : public NrRlc
{
  public:
    NrRlcSm();
    ~NrRlcSm() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoInitialize() override;
    void DoDispose() override;

    void DoTransmitPdcpPdu(Ptr<Packet> p) override;
    void DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams) override;
    void DoNotifyHarqDeliveryFailure() override;
    void DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams) override;

  private:
    /// Buffer status report
    void BufferStatusReport();
};

// /**
//  * Implements NR_RLC Transparent Mode (TM), see  3GPP TS 36.322
//  *
//  */
// class NrRlcTm : public NrRlc
// {
// public:
//   virtual ~NrRlcTm ();

// };

// /**
//  * Implements NR_RLC Unacknowledged Mode (UM), see  3GPP TS 36.322
//  *
//  */
// class NrRlcUm : public NrRlc
// {
// public:
//   virtual ~NrRlcUm ();

// };

// /**
//  * Implements NR_RLC Acknowledged Mode (AM), see  3GPP TS 36.322
//  *
//  */

// class NrRlcAm : public NrRlc
// {
// public:
//   virtual ~NrRlcAm ();
// };

} // namespace ns3

#endif // NR_RLC_H
