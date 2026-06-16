// Copyright (c) 2024 University of Moratuwa
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)

#include "nr-gnb-energy-model.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbEnergyModel");
NS_OBJECT_ENSURE_REGISTERED(NrGnbEnergyModel);

TypeId
NrGnbEnergyModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrGnbEnergyModel")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrGnbEnergyModel>();
    return tid;
}

NrGnbEnergyModel::NrGnbEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

NrGnbEnergyModel::~NrGnbEnergyModel()
{
    NS_LOG_FUNCTION(this);
}

} // namespace ns3
