// Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>,
//         Marco Miozzo <mmiozzo@cttc.es>

#ifndef NR_GNB_CPHY_SAP_H
#define NR_GNB_CPHY_SAP_H

#include "nr-rrc-sap.h"

#include "ns3/ptr.h"

#include <stdint.h>

namespace ns3
{

class NrGnbNetDevice;

/**
 * Service Access Point (SAP) offered by the UE PHY to the UE RRC for control purposes
 *
 * This is the PHY SAP Provider, i.e., the part of the SAP that contains
 * the PHY methods called by the MAC
 */
class NrGnbCphySapProvider
{
  public:
    /**
     * Destructor
     */
    virtual ~NrGnbCphySapProvider() = default;

    /**
     * Set cell ID
     *
     * @param cellId the Cell Identifier
     */
    virtual void SetCellId(uint16_t cellId) = 0;

    /**
     * Set bandwidth
     *
     * @param ulBandwidth the UL bandwidth in PRBs
     * @param dlBandwidth the DL bandwidth in PRBs
     */
    virtual void SetBandwidth(uint16_t ulBandwidth, uint16_t dlBandwidth) = 0;

    /**
     * Set EARFCN
     *
     * @param ulEarfcn the UL EARFCN
     * @param dlEarfcn the DL EARFCN
     */
    virtual void SetEarfcn(uint32_t ulEarfcn, uint32_t dlEarfcn) = 0;

    /**
     * Add a new UE to the cell
     *
     * @param rnti the UE id relative to this cell
     */
    virtual void AddUe(uint16_t rnti) = 0;

    /**
     * Remove an UE from the cell
     *
     * @param rnti the UE id relative to this cell
     */
    virtual void RemoveUe(uint16_t rnti) = 0;

    /**
     * Set the UE transmission power offset P_A
     *
     * @param rnti the UE id relative to this cell
     * @param pa transmission power offset
     */
    virtual void SetPa(uint16_t rnti, double pa) = 0;

    /**
     * Set transmission mode
     *
     * @param rnti the RNTI of the user
     * @param txMode the transmissionMode of the user
     */
    virtual void SetTransmissionMode(uint16_t rnti, uint8_t txMode) = 0;

    /**
     * Set SRS configuration index
     *
     * @param rnti the RNTI of the user
     * @param srsCi the SRS Configuration Index of the user
     */
    virtual void SetSrsConfigurationIndex(uint16_t rnti, uint16_t srsCi) = 0;

    /**
     * Set master information block
     *
     * @param mib the Master Information Block to be sent on the BCH
     */
    virtual void SetMasterInformationBlock(NrRrcSap::MasterInformationBlock mib) = 0;

    /**
     * Set system information block type 1
     *
     * @param sib1 the System Information Block Type 1 to be sent on the BCH
     */
    virtual void SetSystemInformationBlockType1(NrRrcSap::SystemInformationBlockType1 sib1) = 0;

    /**
     * Get reference signal power
     *
     * @return Reference Signal Power for SIB2
     */
    virtual int8_t GetReferenceSignalPower() = 0;

    virtual void AttachUeFromRRC (uint64_t imsi, const Ptr<NetDevice> &netDev) = 0;

    virtual void RegisterUeFromRRC (uint64_t imsi) = 0;

    virtual void SetOptimalGnbBeamForImsi (uint64_t imsi, SfnSf startingSfn, std::vector<uint16_t> optimalBeamIndex, bool isServingCell, uint8_t numOfBeamsTbRlm) = 0;
};

/**
 * Service Access Point (SAP) offered by the UE PHY to the UE RRC for control purposes
 *
 * This is the CPHY SAP User, i.e., the part of the SAP that contains the RRC
 * methods called by the PHY
 */
class NrGnbCphySapUser
{
  public:
    /**
     * Destructor
     */
    virtual ~NrGnbCphySapUser() = default;

    struct UeAssociatedSinrInfo
    {
        uint8_t componentCarrierId;
        std::map<uint64_t, double> ueImsiSinrMap;
    };

    struct UeRRCCSIRSReport
    {
        uint16_t rnti;
        uint8_t cellId;
        std::vector<std::pair<uint8_t, std::pair<double, BeamId>>> csiRSVector;
    };

    virtual void UpdateUeSinrEstimate(NrGnbCphySapUser::UeAssociatedSinrInfo info) = 0;

    virtual uint64_t GetImsiFromRnti (uint16_t rnti) = 0;
};

/**
 * Template for the implementation of the NrGnbCphySapProvider as a member
 * of an owner class of type C to which all methods are forwarded
 */
template <class C>
class MemberNrGnbCphySapProvider : public NrGnbCphySapProvider
{
  public:
    /**
     * Constructor
     *
     * @param owner the owner class
     */
    MemberNrGnbCphySapProvider(C* owner);

    // Delete default constructor to avoid misuse
    MemberNrGnbCphySapProvider() = delete;

    // inherited from NrGnbCphySapProvider
    void SetCellId(uint16_t cellId) override;
    void SetBandwidth(uint16_t ulBandwidth, uint16_t dlBandwidth) override;
    void SetEarfcn(uint32_t ulEarfcn, uint32_t dlEarfcn) override;
    void AddUe(uint16_t rnti) override;
    void RemoveUe(uint16_t rnti) override;
    void SetPa(uint16_t rnti, double pa) override;
    void SetTransmissionMode(uint16_t rnti, uint8_t txMode) override;
    void SetSrsConfigurationIndex(uint16_t rnti, uint16_t srsCi) override;
    void SetMasterInformationBlock(NrRrcSap::MasterInformationBlock mib) override;
    void SetSystemInformationBlockType1(NrRrcSap::SystemInformationBlockType1 sib1) override;
    int8_t GetReferenceSignalPower() override;

    virtual void AttachUeFromRRC (uint64_t imsi, const Ptr<NetDevice> &netDev);
    virtual void RegisterUeFromRRC  (uint64_t imsi);
    virtual void SetOptimalGnbBeamForImsi (uint64_t imsi, SfnSf startingSfn, std::vector<uint16_t> optimalBeamIndex, bool isServingCell, uint8_t numOfBeamsTbRlm);

  private:
    C* m_owner; ///< the owner class
};

template <class C>
MemberNrGnbCphySapProvider<C>::MemberNrGnbCphySapProvider(C* owner)
    : m_owner(owner)
{
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetCellId(uint16_t cellId)
{
    m_owner->DoSetCellId(cellId);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetBandwidth(uint16_t ulBandwidth, uint16_t dlBandwidth)
{
    m_owner->DoSetBandwidth(ulBandwidth, dlBandwidth);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetEarfcn(uint32_t ulEarfcn, uint32_t dlEarfcn)
{
    m_owner->DoSetEarfcn(ulEarfcn, dlEarfcn);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::AddUe(uint16_t rnti)
{
    m_owner->DoAddUe(rnti);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::RemoveUe(uint16_t rnti)
{
    m_owner->DoRemoveUe(rnti);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetPa(uint16_t rnti, double pa)
{
    m_owner->DoSetPa(rnti, pa);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetTransmissionMode(uint16_t rnti, uint8_t txMode)
{
    m_owner->DoSetTransmissionMode(rnti, txMode);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetSrsConfigurationIndex(uint16_t rnti, uint16_t srsCi)
{
    m_owner->DoSetSrsConfigurationIndex(rnti, srsCi);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetMasterInformationBlock(NrRrcSap::MasterInformationBlock mib)
{
    m_owner->DoSetMasterInformationBlock(mib);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetSystemInformationBlockType1(
    NrRrcSap::SystemInformationBlockType1 sib1)
{
    m_owner->DoSetSystemInformationBlockType1(sib1);
}

template <class C>
int8_t
MemberNrGnbCphySapProvider<C>::GetReferenceSignalPower()
{
    return m_owner->DoGetReferenceSignalPower();
}

template <class C>
void 
MemberNrGnbCphySapProvider<C>::AttachUeFromRRC (uint64_t imsi, const Ptr<NetDevice> &netDev)
{
  m_owner->DoAttachUeFromRRC (imsi, netDev);
}

template <class C>
void 
MemberNrGnbCphySapProvider<C>::RegisterUeFromRRC (uint64_t imsi)
{
  m_owner->RegisterUe (imsi);
}

template <class C>
void
MemberNrGnbCphySapProvider<C>::SetOptimalGnbBeamForImsi (uint64_t imsi, SfnSf startingSfn, std::vector<uint16_t> optimalBeamIndex, bool isServingCell, uint8_t numOfBeamsTbRlm)
{
  m_owner->DoSetOptimalGnbBeamForImsi (imsi, startingSfn, optimalBeamIndex, isServingCell, numOfBeamsTbRlm);
}

/**
 * Template for the implementation of the NrGnbCphySapUser as a member
 * of an owner class of type C to which all methods are forwarded
 */
template <class C>
class MemberNrGnbCphySapUser : public NrGnbCphySapUser
{
  public:
    /**
     * Constructor
     *
     * @param owner the owner class
     */
    MemberNrGnbCphySapUser(C* owner);
    virtual void UpdateUeSinrEstimate(UeAssociatedSinrInfo info);
    virtual uint64_t GetImsiFromRnti (uint16_t rnti);

    // Delete default constructor to avoid misuse
    MemberNrGnbCphySapUser() = delete;

    // methods inherited from NrGnbCphySapUser go here

  private:
    C* m_owner; ///< the owner class
};

template <class C>
MemberNrGnbCphySapUser<C>::MemberNrGnbCphySapUser(C* owner)
    : m_owner(owner)
{
}

template <class C>
void
MemberNrGnbCphySapUser<C>::UpdateUeSinrEstimate(UeAssociatedSinrInfo info)
{
  return m_owner->DoUpdateUeSinrEstimate(info);
}

template <class C>
uint64_t
MemberNrGnbCphySapUser<C>::GetImsiFromRnti (uint16_t rnti)
{
  return m_owner->DoGetImsiFromRnti (rnti);
}

} // namespace ns3

#endif // NR_GNB_CPHY_SAP_H
