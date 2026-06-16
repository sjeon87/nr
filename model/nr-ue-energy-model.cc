// Copyright (c) 2024 University of Moratuwa
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)

#include "nr-ue-energy-model.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrUeEnergyModel");
NS_OBJECT_ENSURE_REGISTERED(NrUeEnergyModel);

TypeId
NrUeEnergyModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrUeEnergyModel")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrUeEnergyModel>();
    return tid;
}

NrUeEnergyModel::NrUeEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

NrUeEnergyModel::~NrUeEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

} // namespace ns3
