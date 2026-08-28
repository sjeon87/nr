// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: nipuna dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/nr-ue-phy-energy-listener.h"
#include "ns3/test.h"

/**
 * @file nr-test-ue-phy-energy-listener.cc
 * @ingroup test
 *
 * @brief Unit tests for NrUePhyEnergyListener, the bridge between NR UE PHY
 * events and the UE energy model.
 *
 * At this stage the class is scaffolding: the PHY callbacks are no-ops until
 * the energy model and PHY trace sources are wired in (Week 6). These tests
 * therefore cover the behaviour that is observable today: the object is
 * registered and constructible, and it can be disposed with nothing attached.
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
    }
};

static NrUePhyEnergyListenerTestSuite g_nrUePhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
