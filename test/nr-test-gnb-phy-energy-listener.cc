// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/nr-gnb-phy-energy-listener.h"
#include "ns3/test.h"

/**
 * @file nr-test-gnb-phy-energy-listener.cc
 * @ingroup test
 *
 * @brief Unit tests for NrGnbPhyEnergyListener, the bridge between NR gNB PHY
 * events and the gNB energy model.
 *
 * At this stage the class is scaffolding: the PHY callbacks are no-ops until
 * the energy model and PHY trace sources are wired in (Week 7). These tests
 * therefore cover the behaviour that is observable today: the object is
 * registered and constructible, and the scaling factors (sa, sf, sp) start at
 * the documented safe defaults before any PHY event has occurred.
 */
namespace ns3
{

/**
 * @brief Test that the listener is registered with the TypeId system and can
 *        be constructed both directly and by name.
 */
class NrGnbPhyEnergyListenerCreationTestCase : public TestCase
{
  public:
    NrGnbPhyEnergyListenerCreationTestCase()
        : TestCase("NrGnbPhyEnergyListener construction and TypeId")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerCreationTestCase::DoRun()
{
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr, true, "Listener should be constructible");

    TypeId tid = NrGnbPhyEnergyListener::GetTypeId();
    NS_TEST_ASSERT_MSG_EQ(tid.GetName(),
                          "ns3::NrGnbPhyEnergyListener",
                          "Unexpected registered TypeId name");

    // The class must also resolve through the TypeId/ObjectFactory path.
    TypeId byName = TypeId::LookupByName("ns3::NrGnbPhyEnergyListener");
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
class NrGnbPhyEnergyListenerDefaultsTestCase : public TestCase
{
  public:
    NrGnbPhyEnergyListenerDefaultsTestCase()
        : TestCase("NrGnbPhyEnergyListener default sa, sf, sp values")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerDefaultsTestCase::DoRun()
{
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();

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
class NrGnbPhyEnergyListenerDisposeTestCase : public TestCase
{
  public:
    NrGnbPhyEnergyListenerDisposeTestCase()
        : TestCase("NrGnbPhyEnergyListener dispose with nothing attached")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerDisposeTestCase::DoRun()
{
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    listener->Dispose();
    NS_TEST_ASSERT_MSG_EQ(listener != nullptr,
                          true,
                          "Listener handle should remain valid after Dispose()");
}

/**
 * @brief Test suite for NrGnbPhyEnergyListener.
 */
class NrGnbPhyEnergyListenerTestSuite : public TestSuite
{
  public:
    NrGnbPhyEnergyListenerTestSuite()
        : TestSuite("nr-test-gnb-phy-energy-listener", Type::UNIT)
    {
        AddTestCase(new NrGnbPhyEnergyListenerCreationTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerDefaultsTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerDisposeTestCase(), Duration::QUICK);
    }
};

static NrGnbPhyEnergyListenerTestSuite g_nrGnbPhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
