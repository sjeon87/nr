// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-control-messages.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("nrControlMessage");

NrControlMessage::NrControlMessage()
{
    NS_LOG_FUNCTION(this);
}

NrControlMessage::~NrControlMessage()
{
    NS_LOG_FUNCTION(this);
}

void
NrControlMessage::SetMessageType(messageType type)
{
    m_messageType = type;
}

NrControlMessage::messageType
NrControlMessage::GetMessageType() const
{
    return m_messageType;
}

void
NrControlMessage::SetSourceBwp(uint16_t bwpId)
{
    m_bwpId = bwpId;
}

uint16_t
NrControlMessage::GetSourceBwp() const
{
    NS_ABORT_IF(m_bwpId < 0);
    return static_cast<uint16_t>(m_bwpId);
}

NrSRMessage::NrSRMessage()
{
    NS_LOG_FUNCTION(this);
    SetMessageType(NrControlMessage::SR);
}

NrSRMessage::~NrSRMessage()
{
    NS_LOG_FUNCTION(this);
}

void
NrSRMessage::SetRNTI(uint16_t rnti)
{
    m_rnti = rnti;
}

uint16_t
NrSRMessage::GetRNTI() const
{
    return m_rnti;
}

NrDlDciMessage::NrDlDciMessage(const std::shared_ptr<DciInfoElementTdma>& dci)
    : m_dciInfoElement(dci)
{
    NS_LOG_FUNCTION(this);
    SetMessageType(NrControlMessage::DL_DCI);
}

NrDlDciMessage::~NrDlDciMessage()
{
    NS_LOG_FUNCTION(this);
}

std::shared_ptr<DciInfoElementTdma>
NrDlDciMessage::GetDciInfoElement()
{
    return m_dciInfoElement;
}

void
NrDlDciMessage::SetKDelay(uint32_t delay)
{
    m_k = delay;
}

void
NrDlDciMessage::SetK1Delay(uint32_t delay)
{
    m_k1 = delay;
}

uint32_t
NrDlDciMessage::GetKDelay() const
{
    return m_k;
}

uint32_t
NrDlDciMessage::GetK1Delay() const
{
    return m_k1;
}

NrUlDciMessage::NrUlDciMessage(const std::shared_ptr<DciInfoElementTdma>& dci)
    : m_dciInfoElement(dci)
{
    NS_LOG_FUNCTION(this);
    SetMessageType(NrControlMessage::UL_DCI);
}

NrUlDciMessage::~NrUlDciMessage()
{
    NS_LOG_FUNCTION(this);
}

std::shared_ptr<DciInfoElementTdma>
NrUlDciMessage::GetDciInfoElement()
{
    return m_dciInfoElement;
}

void
NrUlDciMessage::SetKDelay(uint32_t delay)
{
    m_k = delay;
}

uint32_t
NrUlDciMessage::GetKDelay() const
{
    return m_k;
}

NrDlCqiMessage::NrDlCqiMessage()
{
    NS_LOG_FUNCTION(this);
    SetMessageType(NrControlMessage::DL_CQI);
}

NrDlCqiMessage::~NrDlCqiMessage()
{
    NS_LOG_FUNCTION(this);
}

void
NrDlCqiMessage::SetDlCqi(DlCqiInfo cqi)
{
    m_cqi = cqi;
}

DlCqiInfo
NrDlCqiMessage::GetDlCqi()
{
    return m_cqi;
}

// ----------------------------------------------------------------------------------------------------------

NrBsrMessage::NrBsrMessage()
{
    SetMessageType(NrControlMessage::BSR);
}

NrBsrMessage::~NrBsrMessage()
{
}

void
NrBsrMessage::SetBsr(MacCeElement bsr)
{
    m_bsr = bsr;
}

MacCeElement
NrBsrMessage::GetBsr()
{
    return m_bsr;
}

// ----------------------------------------------------------------------------------------------------------

NrMibMessage::NrMibMessage()
{
    SetMessageType(NrControlMessage::MIB);
}

void
NrMibMessage::SetMib(NrRrcSap::MasterInformationBlock mib)
{
    m_mib = mib;
}

NrRrcSap::MasterInformationBlock
NrMibMessage::GetMib() const
{
    return m_mib;
}

// ----------------------------------------------------------------------------------------------------------

NrSib1Message::NrSib1Message()
{
    SetMessageType(NrControlMessage::SIB1);
}

void
NrSib1Message::SetSib1(NrRrcSap::SystemInformationBlockType1 sib1)
{
    m_sib1 = sib1;
}

NrRrcSap::SystemInformationBlockType1
NrSib1Message::GetSib1() const
{
    return m_sib1;
}

// ----------------------------------------------------------------------------------------------------------

NrRachPreambleMessage::NrRachPreambleMessage()
{
    SetMessageType(NrControlMessage::RACH_PREAMBLE);
}

NrRachPreambleMessage::~NrRachPreambleMessage()
{
}

void
NrRachPreambleMessage::SetRapId(uint32_t rapId)
{
    m_rapId = rapId;
}

uint32_t
NrRachPreambleMessage::GetRapId() const
{
    return m_rapId;
}

// ----------------------------------------------------------------------------------------------------------

NrRarMessage::NrRarMessage()
{
    SetMessageType(NrControlMessage::RAR);
}

NrRarMessage::~NrRarMessage()
{
}

void
NrRarMessage::SetRaRnti(uint16_t raRnti)
{
    m_raRnti = raRnti;
}

uint16_t
NrRarMessage::GetRaRnti() const
{
    return m_raRnti;
}

void
NrRarMessage::AddRar(Rar rar)
{
    m_rarList.push_back(rar);
}

std::list<NrRarMessage::Rar>::const_iterator
NrRarMessage::RarListBegin() const
{
    return m_rarList.begin();
}

std::list<NrRarMessage::Rar>::const_iterator
NrRarMessage::RarListEnd() const
{
    return m_rarList.end();
}

NrDlHarqFeedbackMessage::NrDlHarqFeedbackMessage()
{
    SetMessageType(NrControlMessage::DL_HARQ);
}

NrDlHarqFeedbackMessage::~NrDlHarqFeedbackMessage()
{
}

void
NrDlHarqFeedbackMessage::SetDlHarqFeedback(DlHarqInfo m)
{
    m_dlHarqInfo = m;
}

DlHarqInfo
NrDlHarqFeedbackMessage::GetDlHarqFeedback()
{
    return m_dlHarqInfo;
}

NrSrsMessage::NrSrsMessage()
{
    SetMessageType(NrControlMessage::SRS);
}

// ----------------------- MODIFIED -------------------------
NrPssMessage::NrPssMessage (void)
{
  SetMessageType (NrControlMessage::PSS);
}

NrPssMessage::~NrPssMessage()
{

}

void 
NrPssMessage::SetCellId (uint8_t cellId)
{
  m_cellId = cellId;
}

void 
NrPssMessage::SetSNRAvg (double snrAvg)
{
  m_snrAvg = snrAvg;
}

void 
NrPssMessage::SetDestinationImsi (uint64_t destinationImsi)
{
  m_destinationImsi = destinationImsi;
}

void
NrPssMessage::SetSymbolOffset (uint8_t symbolOffset)
{
  m_symbolOffset = symbolOffset;
}

uint8_t
NrPssMessage::GetCellId ()
{
  return m_cellId;
}

double 
NrPssMessage::GetSNRAvg ()
{
  return m_snrAvg;
}

uint64_t
NrPssMessage::GetDestinationImsi ()
{
  return m_destinationImsi;
}

uint8_t
NrPssMessage::GetSymbolOffset ()
{
  return m_symbolOffset;
}

// ----------------------------------------------------------------------------------------------------------

NrSssMessage::NrSssMessage (void)
{
  SetMessageType (NrControlMessage::SSS);
}

NrSssMessage::~NrSssMessage()
{

}

void 
NrSssMessage::SetCellId (uint8_t cellId)
{
  m_cellId = cellId;
}

uint8_t
NrSssMessage::GetCellId ()
{
  return m_cellId;
}

// ----------------------------------------------------------------------------------------------------------

NrPBCHMessage::NrPBCHMessage (void)
{
  SetMessageType (NrPBCHMessage::PBCH_DMRS);
}

NrPBCHMessage::~NrPBCHMessage ()
{

}

void 
NrPBCHMessage::SetCellId (uint8_t cellId)
{
  m_cellId = cellId;
}

uint8_t
NrPBCHMessage::GetCellId ()
{
  return m_cellId;
}

void 
NrPBCHMessage::SetMib (NrRrcSap::MasterInformationBlock mib)
{
  m_mib = mib;
}

NrRrcSap::MasterInformationBlock
NrPBCHMessage::GetMib () 
{
  return m_mib;
}

// ----------------------------------------------------------------------------------------------------------

NRCSIReportMessage::NRCSIReportMessage (void)
{
  SetMessageType (NrControlMessage::CSI_REPORT);
}

NRCSIReportMessage::~NRCSIReportMessage ()
{

}

void 
NRCSIReportMessage::SetCellId (uint8_t cellId)
{
  m_cellId = cellId;
}

uint8_t
NRCSIReportMessage::GetCellId ()
{
  return m_cellId;
}

void 
NRCSIReportMessage::SetRnti (uint16_t rnti)
{
  m_rnti = rnti;
}

uint16_t
NRCSIReportMessage::GetRnti ()
{
  return m_rnti;
}

void 
NRCSIReportMessage::SetCSIRSComponent (uint8_t csiRSRank, std::pair<std::pair<SfnSfKey, uint8_t>, std::pair<BeamId, uint32_t>> csiRSMap)
{
  if (csiRSRank == 0)
  {
    m_firstCSIResource = csiRSMap;
  }
  else if (csiRSRank == 1)
  {
    m_secondCSIResource = csiRSMap;
  }
  else if (csiRSRank == 2)
  {
    m_thirdCSIResource = csiRSMap;
  }
  else if (csiRSRank == 3)
  {
    m_fourthCSIResource = csiRSMap;
  }
  else
  {
    NS_ABORT_MSG ("CSIRSRank should be from 0 to 3");
  }
}

std::pair<std::pair<SfnSfKey, uint8_t>, std::pair<BeamId, uint32_t>> 
NRCSIReportMessage::GetCSIRSMap (uint8_t csiRSRank)
{
  if (csiRSRank == 0)
  {
    return m_firstCSIResource;
  }
  else if (csiRSRank == 1)
  {
    return m_secondCSIResource;
  }
  else if (csiRSRank == 2)
  {
    return m_thirdCSIResource;
  }
  else if (csiRSRank == 3)
  {
    return m_fourthCSIResource;
  }
  else
  {
    NS_ABORT_MSG ("CSIRSRank should be from 0 to 3");
  }
}
// ---------------------------------------------------------

std::ostream&
operator<<(std::ostream& os, const LteNrTddSlotType& item)
{
    switch (item)
    {
    case LteNrTddSlotType::DL:
        os << "DL";
        break;
    case LteNrTddSlotType::F:
        os << "F";
        break;
    case LteNrTddSlotType::S:
        os << "S";
        break;
    case LteNrTddSlotType::UL:
        os << "UL";
        break;
    }
    return os;
}



} // namespace ns3
