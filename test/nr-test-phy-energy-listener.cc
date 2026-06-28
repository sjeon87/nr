// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/nr-phy-energy-listener.h"
#include "ns3/test.h"

/**
 * @file nr-test-phy-energy-listener.cc
 * @ingroup test
 *
 * @brief Unit tests for NrPhyEnergyListener, the bridge between NR PHY events
 * and the energy models.
 *
 * At this stage the class is scaffolding: the PHY callbacks are no-ops until
 * the energy models and PHY trace sources are wired in (Weeks 6-7). These
 * tests therefore cover the behaviour that is observable today: the object is
 * registered and constructible, and the scaling factors (sa, sf, sp) start at
 * the documented safe defaults before any PHY event has occurred.
 */
namespace ns3
{

/**
 * @brief Test that the listener is registered with the TypeId system and can
 *        be constructed both directly and by name.
 */
class NrPhyEnergyListenerCreationTestCase : public TestCase
{
  public:
    NrPhyEnergyListenerCreationTestCase()
        : TestCase("NrPhyEnergyListener construction and TypeId")
    {
    }

  private:
    void DoRun() override;
};

void
NrPhyEnergyListenerCreationTestCase::DoRun()
{
    Ptr<NrPhyEnergyListener> listener = CreateObject<NrPhyEnergyListener>();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr, true, "Listener should be constructible");

    TypeId tid = NrPhyEnergyListener::GetTypeId();
    NS_TEST_ASSERT_MSG_EQ(tid.GetName(),
                          "ns3::NrPhyEnergyListener",
                          "Unexpected registered TypeId name");

    // The class must also resolve through the TypeId/ObjectFactory path.
    TypeId byName = TypeId::LookupByName("ns3::NrPhyEnergyListener");
    NS_TEST_ASSERT_MSG_EQ(byName.GetUid(), tid.GetUid(), "TypeId lookup mismatch");
}

/**
 * @brief Test the documented default scaling factors before any PHY event.
 *
 * Per TR 38.864 Section 5.1 the bridge must report a safe initial state:
 *   - sf = 0  (no RBs allocated yet)
 *   - sp = 1  (full Tx power, current == reference)
 *   - sa = 1  (all antennas active)
 */
class NrPhyEnergyListenerDefaultsTestCase : public TestCase
{
  public:
    NrPhyEnergyListenerDefaultsTestCase()
        : TestCase("NrPhyEnergyListener default sa/sf/sp values")
    {
    }

  private:
    void DoRun() override;
};

void
NrPhyEnergyListenerDefaultsTestCase::DoRun()
{
    Ptr<NrPhyEnergyListener> listener = CreateObject<NrPhyEnergyListener>();

    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastDlSf(),
                              0.0,
                              1e-12,
                              "Default DL sf should be 0 (no allocation yet)");
    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastUlSf(),
                              0.0,
                              1e-12,
                              "Default UL sf should be 0 (no allocation yet)");
    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastSp(),
                              1.0,
                              1e-12,
                              "Default sp should be 1 (full Tx power)");
    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastSa(),
                              1.0,
                              1e-12,
                              "Default sa should be 1 (all antennas active)");
}

/**
 * @brief Test that the listener can be disposed without an attached PHY or
 *        energy model (the "usable without a model installed" contract).
 */
class NrPhyEnergyListenerDisposeTestCase : public TestCase
{
  public:
    NrPhyEnergyListenerDisposeTestCase()
        : TestCase("NrPhyEnergyListener dispose with nothing attached")
    {
    }

  private:
    void DoRun() override;
};

void
NrPhyEnergyListenerDisposeTestCase::DoRun()
{
    Ptr<NrPhyEnergyListener> listener = CreateObject<NrPhyEnergyListener>();
    listener->Dispose();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr,
                          true,
                          "Listener handle should remain valid after Dispose()");
}

/**
 * @brief Test suite for NrPhyEnergyListener.
 */
class NrPhyEnergyListenerTestSuite : public TestSuite
{
  public:
    NrPhyEnergyListenerTestSuite()
        : TestSuite("nr-test-phy-energy-listener", Type::UNIT)
    {
        AddTestCase(new NrPhyEnergyListenerCreationTestCase(), Duration::QUICK);
        AddTestCase(new NrPhyEnergyListenerDefaultsTestCase(), Duration::QUICK);
        AddTestCase(new NrPhyEnergyListenerDisposeTestCase(), Duration::QUICK);
    }
};

static NrPhyEnergyListenerTestSuite g_nrPhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
