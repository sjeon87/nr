// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-mac-scheduler-ai-msg-structs.h"
#include "ns3/test.h"

#include <cstddef>
#include <cstring>

using namespace ns3;

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
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, rnti), 0, "rnti should be at offset 0");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, numLcs), 2, "numLcs should be at offset 2");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, fiveQI), 3, "fiveQI should be at offset 3");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, priority), 4, "priority should be at offset 4");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, holDelay), 6, "holDelay should be at offset 6");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, cqi), 8, "cqi should be at offset 8");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, bsr), 12, "bsr should be at offset 12");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, avgTput), 16, "avgTput should be at offset 16");
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerObservation, potentialTput), 20, "potentialTput should be at offset 20");
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerObservation), 24, "NrSchedulerObservation should be 24 bytes");
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
        NS_TEST_ASSERT_MSG_EQ(offsetof(NrSchedulerAction, weight), 4, "weight should be at offset 4");
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerAction), 8, "NrSchedulerAction should be 8 bytes");
    }
};

class NrSchedEnvelopeSizeTestCase : public TestCase
{
  public:
    NrSchedEnvelopeSizeTestCase()
        : TestCase("Verify envelope structs fit in shared memory")
    {
    }

  private:
    void DoRun() override
    {
        size_t envSize = sizeof(NrSchedulerEnvMessage);
        size_t actSize = sizeof(NrSchedulerActionMessage);
        size_t totalSize = envSize + actSize;

        NS_TEST_ASSERT_MSG_EQ(MAX_UES, 32, "MAX_UES should be 32");

        NS_TEST_ASSERT_MSG_GT(envSize, 0, "NrSchedulerEnvMessage should have nonzero size");
        NS_TEST_ASSERT_MSG_GT(actSize, 0, "NrSchedulerActionMessage should have nonzero size");

        // Both envelopes together should fit in a small shared memory segment (< 64 KB)
        NS_TEST_ASSERT_MSG_LT(totalSize, 65536, "Combined envelope size should be under 64 KB");

        // obs array should hold exactly MAX_UES entries
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerEnvMessage::obs),
                              MAX_UES * sizeof(NrSchedulerObservation),
                              "obs array size mismatch");

        // action array should hold exactly MAX_UES entries
        NS_TEST_ASSERT_MSG_EQ(sizeof(NrSchedulerActionMessage::action),
                              MAX_UES * sizeof(NrSchedulerAction),
                              "action array size mismatch");
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
        // Write known values and verify
        NrSchedulerObservation src{};
        src.rnti = 1001;
        src.numLcs = 3;
        src.fiveQI = 9;
        src.priority = 50;
        src.holDelay = 120;
        src.cqi = 12.5f;
        src.bsr = 256.0f;
        src.avgTput = 1000.0f;
        src.potentialTput = 2000.0f;

        NrSchedulerObservation dst{};
        std::memcpy(&dst, &src, sizeof(NrSchedulerObservation));

        NS_TEST_ASSERT_MSG_EQ(dst.rnti, 1001, "rnti mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.numLcs, 3, "numLcs mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.fiveQI, 9, "fiveQI mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.priority, 50, "priority mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.holDelay, 120, "holDelay mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.cqi, 12.5f, "cqi mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.bsr, 256.0f, "bsr mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.avgTput, 1000.0f, "avgTput mismatch after memcpy");
        NS_TEST_ASSERT_MSG_EQ(dst.potentialTput, 2000.0f, "potentialTput mismatch after memcpy");
    }
};

class NrSchedMsgStructsTestSuite : public TestSuite
{
  public:
    NrSchedMsgStructsTestSuite()
        : TestSuite("nr-mac-scheduler-ai-msg-structs", Type::UNIT)
    {
        AddTestCase(new NrSchedObservationLayoutTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedActionLayoutTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedEnvelopeSizeTestCase(), TestCase::Duration::QUICK);
        AddTestCase(new NrSchedPodTestCase(), TestCase::Duration::QUICK);
    }
};

static NrSchedMsgStructsTestSuite g_nrSchedMsgStructsTestSuite;
