// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/nr-ue-energy-model.h"
#include "ns3/nr-ue-phy-energy-listener.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

/**
 * @file nr-test-ue-phy-energy-listener.cc
 * @ingroup test
 *
 * @brief Unit tests for NrUePhyEnergyListener, the bridge between NR UE PHY
 * events and the UE energy model.
 *
 * Besides the registration / disposal contract, these tests drive the private
 * DL callback directly (the two behavioural test cases are declared friends of
 * the listener) to cover the two spec-consistent refinements added on top of
 * the plain TR 38.840 mapping:
 *   - symbol-accurate active duration: a DL transport block keeps the UE in
 *     full PDCCH+PDSCH reception only for its allocated OFDM symbols, not the
 *     whole slot
 *   - MIMO-rank scaling: the active-reception power is scaled by the number of
 *     spatial layers actually in use (TR 38.840 Section 8.1.3 antenna scaling).
 */
namespace ns3
{

/**
 * @brief Test that the listener is registered with the TypeId system and can
 *        be constructed both directly and by name.
 */
class NrUePhyEnergyListenerCreationTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerCreationTestCase()
        : TestCase("NrUePhyEnergyListener construction and TypeId")
    {
    }

  private:
    void DoRun() override;
};

void
NrUePhyEnergyListenerCreationTestCase::DoRun()
{
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr, true, "Listener should be constructible");

    TypeId tid = NrUePhyEnergyListener::GetTypeId();
    NS_TEST_ASSERT_MSG_EQ(tid.GetName(),
                          "ns3::NrUePhyEnergyListener",
                          "Unexpected registered TypeId name");

    // The class must also resolve through the TypeId/ObjectFactory path.
    TypeId byName = TypeId::LookupByName("ns3::NrUePhyEnergyListener");
    NS_TEST_ASSERT_MSG_EQ(byName.GetUid(), tid.GetUid(), "TypeId lookup mismatch");
}

/**
 * @brief Test that the listener can be disposed without an attached PHY or
 *        energy model (the "usable without a model installed" contract).
 */
class NrUePhyEnergyListenerDisposeTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerDisposeTestCase()
        : TestCase("NrUePhyEnergyListener dispose with nothing attached")
    {
    }

  private:
    void DoRun() override;
};

void
NrUePhyEnergyListenerDisposeTestCase::DoRun()
{
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->Dispose();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr,
                          true,
                          "Listener handle should remain valid after Dispose()");
}

/**
 * @brief A DL transport block must keep the UE in PDCCH+PDSCH reception only for
 *        its allocated OFDM symbols, then fall back to PDCCH-only monitoring.
 *
 * With no PHY attached the listener uses the 1 ms default slot period, so a
 * 2-of-14-symbol allocation stays active for 2/14 ms (~142.857 us). A whole-slot
 * model would still be active at 500 us; the symbol-accurate one is not.
 */
class NrUePhyEnergyListenerDlDurationTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerDlDurationTestCase()
        : TestCase("NrUePhyEnergyListener holds DL reception for the allocated symbols only")
    {
    }

  private:
    void DoRun() override;
    /// Assert the UE is still in full DL reception (sampled mid-allocation).
    void CheckActive();
    /// Assert the UE has fallen back to PDCCH-only (sampled after the allocation).
    void CheckReturned();

    Ptr<NrUeEnergyModel> m_model; //!< model driven by the listener under test
};

void
NrUePhyEnergyListenerDlDurationTestCase::CheckActive()
{
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_PDSCH,
                          "UE must stay in PDCCH+PDSCH for the allocated symbols");
}

void
NrUePhyEnergyListenerDlDurationTestCase::CheckReturned()
{
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_ONLY,
                          "UE must return to PDCCH-only after the allocated symbols, "
                          "not hold the whole slot");
}

void
NrUePhyEnergyListenerDlDurationTestCase::DoRun()
{
    m_model = CreateObject<NrUeEnergyModel>();
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->SetEnergyModel(m_model);
    // No SetPhy(): slot period stays at the 1 ms default and BWP refresh is a no-op.

    // DL TB using 2 of the 14 symbols at t = 0 -> active for 2/14 ms (~142.857 us).
    listener->DlTbReceivedCallback(/*imsi*/ 1,
                                   /*tbSize*/ 100,
                                   /*symStart*/ 0,
                                   /*numSym*/ 2,
                                   /*rank*/ 1);
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_PDSCH,
                          "DL TB should move the UE to full PDCCH+PDSCH reception");

    // 50 us: inside the 2-symbol window -> still active.
    Simulator::Schedule(NanoSeconds(50000),
                        &NrUePhyEnergyListenerDlDurationTestCase::CheckActive,
                        this);
    // 500 us: past the 2-symbol window but still inside the 1 ms slot -> a
    // whole-slot model would be active here, the symbol-accurate one is not.
    Simulator::Schedule(NanoSeconds(500000),
                        &NrUePhyEnergyListenerDlDurationTestCase::CheckReturned,
                        this);
    Simulator::Run();
    Simulator::Destroy();
    m_model = nullptr;
}

/**
 * @brief A DL transport block must scale the UE active-reception power by the
 *        number of MIMO layers (rank) actually in use.
 *
 * With the reference receive-antenna count at its default of 4, a full-rank
 * (rank 4) TB draws the reference power, and a rank-1 TB draws 0.7^2 = 0.49 of
 * it (one 0.7 factor per halving of the active receive chains, 4 -> 2 -> 1).
 */
class NrUePhyEnergyListenerRankScalingTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerRankScalingTestCase()
        : TestCase("NrUePhyEnergyListener scales UE active power by MIMO rank")
    {
    }

  private:
    void DoRun() override;
};

void
NrUePhyEnergyListenerRankScalingTestCase::DoRun()
{
    Ptr<NrUeEnergyModel> model = CreateObject<NrUeEnergyModel>();
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->SetEnergyModel(model);

    // Full-rank DL TB: ApplyAntennaScaling(4) with the reference of 4 -> factor 1.0.
    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 4);
    double pFull = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    // Rank-1 DL TB: ApplyAntennaScaling(1) -> two halvings from 4 -> 0.7^2 = 0.49.
    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 1);
    double pRank1 = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    NS_TEST_ASSERT_MSG_GT(pFull, 0.0, "reference active power must be positive");
    NS_TEST_ASSERT_MSG_EQ_TOL(pRank1 / pFull,
                              0.49,
                              1e-9,
                              "rank-1 active power must be 0.7^2 of the full-rank power");
    Simulator::Destroy();
}

/**
 * @brief Test suite for NrUePhyEnergyListener.
 */
class NrUePhyEnergyListenerTestSuite : public TestSuite
{
  public:
    NrUePhyEnergyListenerTestSuite()
        : TestSuite("nr-test-ue-phy-energy-listener", Type::UNIT)
    {
        AddTestCase(new NrUePhyEnergyListenerCreationTestCase(), Duration::QUICK);
        AddTestCase(new NrUePhyEnergyListenerDisposeTestCase(), Duration::QUICK);
        AddTestCase(new NrUePhyEnergyListenerDlDurationTestCase(), Duration::QUICK);
        AddTestCase(new NrUePhyEnergyListenerRankScalingTestCase(), Duration::QUICK);
    }
};

static NrUePhyEnergyListenerTestSuite g_nrUePhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
