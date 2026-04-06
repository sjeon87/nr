// Copyright (c) 2026 University of Moratuwa
// Author: Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#if __has_include("ns3/ai-module.h")
#define HAVE_NS3_AI
#include "nr-mac-scheduler-ai-msg-structs.h"
#include "nr-mac-scheduler-ue-info-ai.h"
#include "ns3/ai-module.h"
#include "ns3/object.h"
#include <string>
#include <vector>
namespace ns3 {
/**
 * @ingroup scheduler
 * @brief Bridge between the NR AI scheduler and ns-3-ai msg interface.
 *
 * This class implements the same NotifyCb callback signature as
 * NrMacSchedulerAiNs3GymEnv, but communicates with a Python agent
 * through Boost.Interprocess shared memory instead of ZMQ sockets.
 *
 * Per-TTI data flow:
 *   1. C++ packs observations + reward into NrSchedEnvMsg (shared mem)
 *   2. Semaphore signals Python
 *   3. Python reads observations, runs forward pass, writes NrSchedActMsg
 *   4. Semaphore signals C++
 *   5. C++ reads weights, applies them via UpdateAllUeWeightsFn
 *
 *
 * @see NrMacSchedulerAiNs3GymEnv  (the OpenGym counterpart)
 * @see NrSchedEnvMsg
 * @see NrSchedActMsg
 */
class NrMacSchedulerAiMsgEnv : public Object {
public:
  /**
   * @brief Default constructor.
   *
   * Initialises the shared memory segment via Ns3AiMsgInterface.
   * The C++ side is the memory creator. Python opens the existing
   * segment.
   */
  NrMacSchedulerAiMsgEnv();
  /**
   * @brief Constructor with explicit flow count.
   * @param numFlows expected maximum number of active flows
   *
   * The numFlows parameter is informational; the actual number of
   * observations written each TTI may be smaller.
   */
  explicit NrMacSchedulerAiMsgEnv(uint32_t numFlows);
  /**
   * @brief Destructor signals Python that the simulation is over.
   */
  ~NrMacSchedulerAiMsgEnv() override;
  /**
   * @brief Get the ns-3 TypeId for this class.
   * @return the TypeId
   */
  static TypeId GetTypeId();
  /**
   * @brief Dispose of the object and release shared memory.
   */
  void DoDispose() override;
  /**
   * @brief Notify the environment about the current scheduling iteration.
   * @param observations perflow observations from the scheduler
   * @param isGameOver   whether the simulation episode has ended
   * @param reward       scalar reward for the previous action
   * @param extraInfo    optional metadata string
   * @param updateAllUeWeightsFn callback to apply weights to the scheduler
   *
   * This method is called once per TTI by CallNotifyDlFn / CallNotifyUlFn
   * in NrMacSchedulerOfdmaAi or NrMacSchedulerTdmaAi.  It performs the
   * full shared-memory send/receive cycle synchronously: observations are
   * written, Python is signalled, C++ blocks until Python writes back
   * the scheduling weights, and then the weights are applied.
   */
  void NotifyCurrentIteration(
      const std::vector<NrMacSchedulerUeInfoAi::LcObservation> &observations,
      bool isGameOver, float reward, const std::string &extraInfo,
      const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn &updateAllUeWeightsFn);

private:
  uint32_t m_numFlows{0}; ///< Expected number of active flows
  /**
   * @brief Pointer to the msg interface implementation.
   *
   * Owned by the Ns3AiMsgInterface singleton; this is a non owning
   * pointer obtained via GetInterface<>().
   */
  Ns3AiMsgInterfaceImpl<NrSchedEnvMsg, NrSchedActMsg> *m_msgInterface{nullptr};
};
} // namespace ns3
#endif // HAVE_NS3_AI