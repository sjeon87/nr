// Copyright (c) 2024 University of Moratuwa
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.840 V16.0.0 (2019-06): Section 8 - UE energy consumption evaluation

#ifndef NR_UE_ENERGY_MODEL_H
#define NR_UE_ENERGY_MODEL_H

#include "ns3/object.h"

namespace ns3
{

/**
 * @ingroup nr
 * @brief Placeholder for the TR 38.840 UE energy model (Week 4).
 *
 * Minimal stub so that NrPhyEnergyListener has a complete type to hold and
 * drive. The UE power-state machine, scaling rules, and energy accounting
 * are added in a later milestone.
 */
class NrUeEnergyModel : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrUeEnergyModel constructor
     */
    NrUeEnergyModel();

    /**
     * @brief ~NrUeEnergyModel
     */
    ~NrUeEnergyModel() override;
};

} // namespace ns3

#endif // NR_UE_ENERGY_MODEL_H
