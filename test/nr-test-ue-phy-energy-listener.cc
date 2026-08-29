// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/boolean.h"
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
 * DL callback directly (the behavioural test cases are declared friends of the
 * listener) to cover two properties of the TR 38.840 mapping:
 *   - slot-averaged active duration: Table 18/20 powers are already averaged over
 *     the operations within a slot, so a transport block holds the active state
 *     for the whole slot rather than for its allocated OFDM symbols
 *   - receive-chain scaling: the Table 21 antenna scaling comes from the static
 *     NrUeEnergyModel::ActiveRxChains configuration, and the reported MIMO rank
 *     does not touch it unless UseRankAsRxChains is set.
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
 * @brief A DL transport block holds the active state for the whole slot.
 *
 * TR 38.840 Table 18 powers are averaged over the operations within a slot
 * (Release 16, p.63)
 */
class NrUePhyEnergyListenerSlotAveragedDurationTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerSlotAveragedDurationTestCase()
        : TestCase("NrUePhyEnergyListener holds the slot-averaged power for the whole slot")
    {
    }

  private:
    void DoRun() override;
    /// Assert the UE is still in full DL reception (sampled inside the slot).
    void CheckActive();
    /// Assert the UE has fallen back to PDCCH-only (sampled after the slot).
    void CheckReturned();

    Ptr<NrUeEnergyModel> m_model; //!< model driven by the listener under test
};

void
NrUePhyEnergyListenerSlotAveragedDurationTestCase::CheckActive()
{
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_PDSCH,
                          "slot-averaged Table 18 power must be held for the whole slot");
}

void
NrUePhyEnergyListenerSlotAveragedDurationTestCase::CheckReturned()
{
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_ONLY,
                          "UE must return to PDCCH-only after the slot");
}

void
NrUePhyEnergyListenerSlotAveragedDurationTestCase::DoRun()
{
    m_model = CreateObject<NrUeEnergyModel>();
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->SetEnergyModel(m_model);
    // No SetPhy(): slot period stays at the 1 ms default and BWP refresh is a no-op.

    // DL TB using 2 of the 14 symbols at t = 0 -> active for the whole 1 ms slot.
    listener->DlTbReceivedCallback(/*imsi*/ 1,
                                   /*tbSize*/ 100,
                                   /*symStart*/ 0,
                                   /*numSym*/ 2,
                                   /*rank*/ 1);
    NS_TEST_ASSERT_MSG_EQ(m_model->GetCurrentState(),
                          NR_UE_PDCCH_PDSCH,
                          "DL TB should move the UE to full PDCCH+PDSCH reception");

    // 500 us: past the 2-symbol window but inside the slot -> still active. A
    // numSym-proportional model would already have returned to monitoring here.
    Simulator::Schedule(NanoSeconds(500000),
                        &NrUePhyEnergyListenerSlotAveragedDurationTestCase::CheckActive,
                        this);
    // 1.001 ms: just past the slot -> back to PDCCH-only monitoring.
    Simulator::Schedule(NanoSeconds(1001000),
                        &NrUePhyEnergyListenerSlotAveragedDurationTestCase::CheckReturned,
                        this);
    Simulator::Run();
    Simulator::Destroy();
    m_model = nullptr;
}

/**
 * @brief By default the reported MIMO rank must NOT change the UE active power.
 *
 * TR 38.840 Table 21 scales with powered receive chains, not spatial layers, so
 * a rank-1 TB must draw exactly what a rank-4 one draws.
 */
class NrUePhyEnergyListenerRankDefaultOffTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerRankDefaultOffTestCase()
        : TestCase("NrUePhyEnergyListener leaves the antenna scaling alone by default")
    {
    }

  private:
    void DoRun() override;
};

void
NrUePhyEnergyListenerRankDefaultOffTestCase::DoRun()
{
    Ptr<NrUeEnergyModel> model = CreateObject<NrUeEnergyModel>();
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->SetEnergyModel(model);
    model->Initialize();

    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 4);
    double pRank4 = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 1);
    double pRank1 = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    NS_TEST_ASSERT_MSG_GT(pRank4, 0.0, "active power must be positive");
    NS_TEST_ASSERT_MSG_EQ_TOL(pRank1,
                              pRank4,
                              1e-12,
                              "rank must not scale the active power by default");
    Simulator::Destroy();
}

/**
 * @brief With UseRankAsRxChains set, rank drives the Table 21 scaling.
 *
 * Opt-in, explicitly non-3GPP: against the FR1 reference of 4 chains, rank 1
 * applies two 0.7 factors (4 -> 2 -> 1).
 */
class NrUePhyEnergyListenerRankOptInTestCase : public TestCase
{
  public:
    NrUePhyEnergyListenerRankOptInTestCase()
        : TestCase("NrUePhyEnergyListener maps rank onto Table 21 when opted in")
    {
    }

  private:
    void DoRun() override;
};

void
NrUePhyEnergyListenerRankOptInTestCase::DoRun()
{
    Ptr<NrUeEnergyModel> model = CreateObject<NrUeEnergyModel>();
    Ptr<NrUePhyEnergyListener> listener = CreateObject<NrUePhyEnergyListener>();
    listener->SetAttribute("UseRankAsRxChains", BooleanValue(true));
    listener->SetEnergyModel(model);
    model->Initialize();

    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 4);
    double pRank4 = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    listener->DlTbReceivedCallback(1, 100, 0, 14, /*rank*/ 1);
    double pRank1 = model->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    NS_TEST_ASSERT_MSG_GT(pRank4, 0.0, "active power must be positive");
    NS_TEST_ASSERT_MSG_EQ_TOL(pRank1 / pRank4,
                              0.49,
                              1e-9,
                              "opted-in rank-1 power must be 0.7^2 of the rank-4 power");
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
        AddTestCase(new NrUePhyEnergyListenerSlotAveragedDurationTestCase(), Duration::QUICK);
        AddTestCase(new NrUePhyEnergyListenerRankDefaultOffTestCase(), Duration::QUICK);
        AddTestCase(new NrUePhyEnergyListenerRankOptInTestCase(), Duration::QUICK);
    }
};

static NrUePhyEnergyListenerTestSuite g_nrUePhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
