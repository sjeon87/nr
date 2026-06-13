// Copyright (c) 2026 University of Peradeniya (UOP)
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-mac-scheduler-ai-msg-structs.h"
#include "ns3/test.h"

#include <cstddef>
#include <cstring>
#include <type_traits>

using namespace ns3;

// Compile-time guarantee: the shared-memory transport relies on these structs
// being trivially copyable PODs.
static_assert(std::is_trivially_copyable<NrSchedulerLcObservation>::value,
              "NrSchedulerLcObservation must be trivially copyable");
static_assert(std::is_trivially_copyable<NrSchedulerObservation>::value,
              "NrSchedulerObservation must be trivially copyable");
static_assert(std::is_trivially_copyable<NrSchedulerAction>::value,
              "NrSchedulerAction must be trivially copyable");

class NrSchedLcObservationLayoutTestCase : public TestCase
{
  public:
    NrSchedLcObservationLayoutTestCase()
        : TestCase("Verify NrSchedulerLcObservation field offsets and size")
    {
    }

  private:
    void DoRun() override
    {
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, holDelay),
                              0,
                              "holDelay should be at offset 0");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, delayBudgetMs),
                              2,
                              "delayBudgetMs should be at offset 2");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, lcId),
                              4,
                              "lcId should be at offset 4");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, fiveQI),
                              5,
                              "fiveQI should be at offset 5");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, priority),
                              6,
                              "priority should be at offset 6");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, resourceType),
                              7,
                              "resourceType should be at offset 7");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerLcObservation, bsr),
                              8,
                              "bsr should be at offset 8");
        // 12 bytes with every byte accounted for: no hidden padding
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerLcObservation),
                              12,
                              "NrSchedulerLcObservation should be 12 bytes");
    }
};

class NrSchedObservationLayoutTestCase : public TestCase
{
  public:
    NrSchedObservationLayoutTestCase()
        : TestCase("Verify NrSchedulerObservation field offsets and size")
    {
    }

  private:
    void DoRun() override
    {
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, cqi),
                              0,
                              "cqi should be at offset 0");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, avgTput),
                              4,
                              "avgTput should be at offset 4");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, potentialTput),
                              8,
                              "potentialTput should be at offset 8");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, assignedBytes),
                              12,
                              "assignedBytes should be at offset 12");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, rnti),
                              16,
                              "rnti should be at offset 16");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, numLcs),
                              18,
                              "numLcs should be at offset 18");
        // one reserved padding byte at offset 19
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, lc),
                              20,
                              "lc array should be at offset 20");
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerObservation::lc),
                              MAX_LCS_PER_UE * sizeof(NrSchedulerLcObservation),
                              "lc array size mismatch");
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerObservation),
                              68,
                              "NrSchedulerObservation should be 68 bytes");
    }
};

class NrSchedActionLayoutTestCase : public TestCase
{
  public:
    NrSchedActionLayoutTestCase()
        : TestCase("Verify NrSchedulerAction field offsets and size")
    {
    }

  private:
    void DoRun() override
    {
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerAction, rnti), 0, "rnti should be at offset 0");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerAction, weight),
                              4,
                              "weight should be at offset 4");
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerAction), 8, "NrSchedulerAction should be 8 bytes");
    }
};

class NrSchedVectorElementTestCase : public TestCase
{
  public:
    NrSchedVectorElementTestCase()
        : TestCase("Verify vector element structs are small and trivially copyable")
    {
    }

  private:
    void DoRun() override
    {
        // The structs are pushed as elements of an ns3-ai message-interface
        // vector, so they must be trivially copyable POD types.
        NS_TEST_ASSERT_MSG_EQ(std::is_trivially_copyable<NrSchedulerLcObservation>::value,
                              true,
                              "NrSchedulerLcObservation must be trivially copyable");
        NS_TEST_ASSERT_MSG_EQ(std::is_trivially_copyable<NrSchedulerObservation>::value,
                              true,
                              "NrSchedulerObservation must be trivially copyable");
        NS_TEST_ASSERT_MSG_EQ(std::is_trivially_copyable<NrSchedulerAction>::value,
                              true,
                              "NrSchedulerAction must be trivially copyable");

        NS_TEST_ASSERT_MSG_EQ(MAX_LCS_PER_UE, 4, "MAX_LCS_PER_UE should be 4");

        // Each per-UE element should stay small (cheap to copy in shared memory)
        NS_TEST_ASSERT_MSG_LT(sizeof(NrSchedulerObservation), 256, "observation element too large");
        NS_TEST_ASSERT_MSG_LT(sizeof(NrSchedulerAction), 64, "action element too large");
    }
};

class NrSchedPodTestCase : public TestCase
{
  public:
    NrSchedPodTestCase()
        : TestCase("Verify structs are memcpy-safe (POD behaviour)")
    {
    }

  private:
    void DoRun() override
    {
        // Write known values, including a per-bearer entry, and verify
        NrSchedulerObservation src{};
        src.cqi = 12.5f;
        src.avgTput = 1000.0f;
        src.potentialTput = 2000.0f;
        src.assignedBytes = 4321;
        src.rnti = 1001;
        src.numLcs = 2;
        src.lc[0].holDelay = 120;
        src.lc[0].delayBudgetMs = 300;
        src.lc[0].lcId = 3;
        src.lc[0].fiveQI = 9;
        src.lc[0].priority = 50;
        src.lc[0].resourceType = 1;
        src.lc[0].bsr = 256.0f;

        NrSchedulerObservation dst{};
        std::memcpy(&dst, &src, sizeof(NrSchedulerObservation));

        NS_TEST_ASSERT_MSG_EQ(dst.cqi, 12.5f, "cqi mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.avgTput, 1000.0f, "avgTput mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.potentialTput, 2000.0f, "potentialTput mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.assignedBytes, 4321, "assignedBytes mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.rnti, 1001, "rnti mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.numLcs, 2, "numLcs mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].holDelay, 120, "lc[0].holDelay mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].delayBudgetMs,
                              300,
                              "lc[0].delayBudgetMs mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].lcId, 3, "lc[0].lcId mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].fiveQI, 9, "lc[0].fiveQI mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].priority, 50, "lc[0].priority mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].resourceType,
                              1,
                              "lc[0].resourceType mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.lc[0].bsr, 256.0f, "lc[0].bsr mismatch after memcpy");
    }
};

class NrSchedMsgStructsTestSuite : public TestSuite
{
  public:
    NrSchedMsgStructsTestSuite()
        : TestSuite("nr-mac-scheduler-ai-msg-structs", Type::UNIT)
    {
        AddTestCase(new NrSchedLcObservationLayoutTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedObservationLayoutTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedActionLayoutTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedVectorElementTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedPodTestCase(), TestCase::Duration::QUICK);
    }
};

static NrSchedMsgStructsTestSuite g_nrSchedMsgStructsTestSuite;
