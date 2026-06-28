// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5 - Energy consumption model for BS

#ifndef NR_GNB_ENERGY_MODEL_H
#define NR_GNB_ENERGY_MODEL_H

#include "ns3/object.h"

namespace ns3
{

/**
 * @ingroup nr
 * @brief Placeholder for the TR 38.864 gNB energy model (Week 5).
 *
 * Minimal stub so that NrPhyEnergyListener has a complete type to hold and
 * drive. The gNB power-state machine, DL/UL power formulas, and symbol-level
 * energy accounting are added in a later milestone.
 */
class NrGnbEnergyModel : public Object
{
  public:
    /**
     * @brief Get the TypeId
     * @return the TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief NrGnbEnergyModel constructor
     */
    NrGnbEnergyModel();

    /**
     * @brief ~NrGnbEnergyModel
     */
    ~NrGnbEnergyModel() override;
};

} // namespace ns3

#endif // NR_GNB_ENERGY_MODEL_H
