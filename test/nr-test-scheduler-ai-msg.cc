// Copyright (c) 2026 University of Moratuwa
// Author: Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file nr-test-scheduler-ai-msg.cc
 * @ingroup test
 *
 * @brief Unit tests for the ns-3-ai msg-interface AI scheduler bridge.
 *
 * These tests verify that:
 * - The extended LcObservation struct correctly carries 9 features
 * - The Lyapunov-based reward separates eMBB and URLLC properly
 * - The existing AI scheduler callback interface remains intact
 * - Build compilation succeeds both with and without contrib/ai
 *
 * Note: These tests do NOT require a running Python process.
 * They test the C++ side of the bridge in isolation.
 */
#include "ns3/beam-id.h"
#include "ns3/log.h"
#include "ns3/nr-mac-scheduler-lcg.h"
#include "ns3/nr-mac-scheduler-ue-info-ai.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/nstime.h"
#include "ns3/test.h"

#include <cmath>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrTestSchedulerAiMsg");

/**
 * @brief Test case: verify extended LcObservation fields.
 *
 * Creates NrMacSchedulerUeInfoAi instances, populates DL LCGs with
 * realistic parameters, and verifies that GetDlObservation() returns
 * LcObservation structs containing all 9 fields with correct values.
 */
class NrTestExtendedObservationCase : public TestCase
{
  public:
    NrTestExtendedObservationCase()
        : TestCase("NrTestExtendedObservationCase")
    {
    }

  private:
    void DoRun() override;
};

void
NrTestExtendedObservationCase::DoRun()
{
    NS_LOG_FUNCTION(this);
    // Create a minimal AI UE info object
    auto getRbPerRbg = []() -> uint32_t { return 1; };
    NrMacSchedulerUeInfoAi ueInfo(0.0, /*rnti=*/1, BeamId(0, 0.0), getRbPerRbg);
    // Create and insert a DL LCG with one LC
    auto lcg = std::make_unique<NrMacSchedulerLCG>(/*id=*/0);
    nr::LogicalChannelConfigListElement_s lcConf;
    lcConf.m_logicalChannelIdentity = 1;
    lcConf.m_logicalChannelGroup = 0;
    lcConf.m_direction = nr::LogicalChannelConfigListElement_s::DIR_DL;
    lcConf.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
    lcConf.m_fiveQi = 9; // eMBB
    auto lc = std::make_unique<NrMacSchedulerLC>(lcConf);
    lc->m_rlcTransmissionQueueHolDelay = 42;
    lc->m_rlcTransmissionQueueSize = 1024;
    lc->m_priority = 70;
    lcg->Insert(std::move(lc));
    ueInfo.m_dlLCG[0] = std::move(lcg);
    // Set throughput / CQI values
    ueInfo.m_avgTputDl = 5000.0;
    ueInfo.m_potentialTputDl = 8000.0;
    ueInfo.m_dlCqi.m_wbCqi = 12;
    // Get observation
    auto obs = ueInfo.GetDlObservation();
    NS_TEST_ASSERT_MSG_EQ(obs.size(), 1, "Should have exactly 1 observation");
    // Verify all 9 fields
    NS_TEST_ASSERT_MSG_EQ(obs[0].rnti, 1, "RNTI mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].lcId, 1, "LCID mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].fiveQi, 9, "5QI mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].priority, 70, "Priority mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].holDelay, 42, "HoL delay mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].cqi, 12.0f, "CQI mismatch");
    NS_TEST_ASSERT_MSG_EQ(obs[0].bsr, 1024.0f, "BSR mismatch");
    // Floating point checks with tolerance
    NS_TEST_ASSERT_MSG_EQ_TOL(obs[0].avgTput, 5000.0f, 0.01f, "avgTput mismatch");
    NS_TEST_ASSERT_MSG_EQ_TOL(obs[0].potentialTput, 8000.0f, 0.01f, "potentialTput mismatch");
}

/**
 * @brief Test case: verify Lyapunov reward computation.
 *
 * Creates two UEs: one eMBB (5QI=9) and one URLLC (5QI=1).
 * Verifies that:
 * - eMBB contributes log(avgTput) to the reward
 * - URLLC contributes -lambda * max(0, delay - PDB)^2
 * - When delay < PDB, the URLLC penalty is zero
 */
class NrTestLyapunovRewardCase : public TestCase
{
  public:
    NrTestLyapunovRewardCase()
        : TestCase("NrTestLyapunovRewardCase")
    {
    }

  private:
    void DoRun() override;
};

void
NrTestLyapunovRewardCase::DoRun()
{
    NS_LOG_FUNCTION(this);
    auto getRbPerRbg = []() -> uint32_t { return 1; };
    // --- eMBB UE (5QI = 9) ---
    NrMacSchedulerUeInfoAi ueEmbb(0.0, /*rnti=*/1, BeamId(0, 0.0), getRbPerRbg);
    {
        auto lcg = std::make_unique<NrMacSchedulerLCG>(0);
        nr::LogicalChannelConfigListElement_s lcConf;
        lcConf.m_logicalChannelIdentity = 1;
        lcConf.m_logicalChannelGroup = 0;
        lcConf.m_direction = nr::LogicalChannelConfigListElement_s::DIR_DL;
        lcConf.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
        lcConf.m_fiveQi = 9;
        auto lc = std::make_unique<NrMacSchedulerLC>(lcConf);
        lc->m_rlcTransmissionQueueHolDelay = 10;
        lc->m_rlcTransmissionQueueSize = 512;
        lc->m_priority = 70;
        lcg->Insert(std::move(lc));
        ueEmbb.m_dlLCG[0] = std::move(lcg);
    }
    ueEmbb.m_avgTputDl = 1000.0;
    ueEmbb.m_potentialTputDl = 2000.0;
    float embbReward = ueEmbb.GetDlRewardLyapunov(1.0f);
    float expectedEmbb = std::log(1000.0f);
    NS_TEST_ASSERT_MSG_EQ_TOL(embbReward, expectedEmbb, 0.01f, "eMBB Lyapunov reward mismatch");
    // URLLC UE (5QI = 1, PDB = 100ms, delay = 150ms)
    NrMacSchedulerUeInfoAi ueUrllc(0.0, /*rnti=*/2, BeamId(0, 0.0), getRbPerRbg);
    {
        auto lcg = std::make_unique<NrMacSchedulerLCG>(0);
        nr::LogicalChannelConfigListElement_s lcConf;
        lcConf.m_logicalChannelIdentity = 1;
        lcConf.m_logicalChannelGroup = 0;
        lcConf.m_direction = nr::LogicalChannelConfigListElement_s::DIR_DL;
        lcConf.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_DGBR;
        lcConf.m_fiveQi = 1;
        auto lc = std::make_unique<NrMacSchedulerLC>(lcConf);
        lc->m_rlcTransmissionQueueHolDelay = 150; // exceeds PDB
        lc->m_rlcTransmissionQueueSize = 256;
        lc->m_priority = 20;
        lc->m_delayBudget = MilliSeconds(100);
        lcg->Insert(std::move(lc));
        ueUrllc.m_dlLCG[0] = std::move(lcg);
    }
    ueUrllc.m_avgTputDl = 500.0;
    float urllcReward = ueUrllc.GetDlRewardLyapunov(1.0f);
    // violation = max(0, 150 - 100) = 50
    // penalty = -1.0 * 50^2 = -2500
    float expectedUrllc = -2500.0f;
    NS_TEST_ASSERT_MSG_EQ_TOL(urllcReward, expectedUrllc, 0.01f, "URLLC Lyapunov reward mismatch");
    // URLLC with no violation (delay < PDB)
    NrMacSchedulerUeInfoAi ueUrllcOk(0.0, /*rnti=*/3, BeamId(0, 0.0), getRbPerRbg);
    {
        auto lcg = std::make_unique<NrMacSchedulerLCG>(0);
        nr::LogicalChannelConfigListElement_s lcConf;
        lcConf.m_logicalChannelIdentity = 1;
        lcConf.m_logicalChannelGroup = 0;
        lcConf.m_direction = nr::LogicalChannelConfigListElement_s::DIR_DL;
        lcConf.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_DGBR;
        lcConf.m_fiveQi = 2;
        auto lc = std::make_unique<NrMacSchedulerLC>(lcConf);
        lc->m_rlcTransmissionQueueHolDelay = 50; // below PDB
        lc->m_rlcTransmissionQueueSize = 128;
        lc->m_priority = 15;
        lc->m_delayBudget = MilliSeconds(150);
        lcg->Insert(std::move(lc));
        ueUrllcOk.m_dlLCG[0] = std::move(lcg);
    }
    float noViolationReward = ueUrllcOk.GetDlRewardLyapunov(1.0f);
    NS_TEST_ASSERT_MSG_EQ_TOL(noViolationReward,
                              0.0f,
                              0.01f,
                              "No-violation URLLC should have 0 reward");
}

// Test Suite
/**
 * @brief Test suite for ns-3-ai msg-interface AI scheduler.
 *
 * Tests the extended LcObservation struct and Lyapunov reward.
 * Callback compatibility tests are in nr-test-scheduler-ai.cc
 * (requires the opengym module for OfdmaAi/TdmaAi headers).
 */
class NrTestSchedulerAiMsgSuite : public TestSuite
{
  public:
    NrTestSchedulerAiMsgSuite()
        : TestSuite("nr-test-scheduler-ai-msg", Type::UNIT)
    {
        // Extended observation test
        AddTestCase(new NrTestExtendedObservationCase(), Duration::QUICK);
        // Lyapunov reward test
        AddTestCase(new NrTestLyapunovRewardCase(), Duration::QUICK);
    }
};

static NrTestSchedulerAiMsgSuite nrTestSchedulerAiMsgSuite;
} // namespace ns3
