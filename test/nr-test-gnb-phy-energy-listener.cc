// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/double.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-gnb-phy-energy-listener.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <cmath>

/**
 * @file nr-test-gnb-phy-energy-listener.cc
 * @ingroup test
 *
 * @brief Unit tests for NrGnbPhyEnergyListener, the bridge between NR gNB PHY
 * events and the gNB energy model.
 *
 * The listener owns no 3GPP arithmetic: it measures what one slot occupied and
 * hands it to NrGnbEnergyModel, which applies the TR 38.864 formulas. These
 * tests therefore drive one slot through SlotEnergyStatsCallback() with known
 * statistics and check what the listener derived from it - sf, sp, the symbol
 * classification and the resulting slot power - alongside the construction and
 * teardown cases.
 *
 * The PHY here is a real NrGnbPhy, configured only as far as the listener reads
 * it: bandwidth and numerology give the RB count and the symbol period, the Tx
 * power feeds sp, and the TDD pattern decides whether the bandwidth part counts
 * towards the carrier's downlink reference bandwidth.
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
 * @brief Shared fixture: a real gNB PHY, configured as far as the listener reads it.
 *
 * SetNumerology() fixes the symbol and slot periods, and SetTxPower() and
 * SetPattern() supply the other two things the listener asks the PHY for. The
 * channel bandwidth is deliberately not set - there is no public way to do so on
 * a bare PHY, so GetRbNum() stays zero and the carrier's reference bandwidth is
 * supplied through the model's ReferenceRbCount attribute instead, which is the
 * documented way to state it explicitly.
 */
class NrGnbPhyEnergyListenerSlotTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param name Test case name.
     */
    NrGnbPhyEnergyListenerSlotTestCase(std::string name)
        : TestCase(name)
    {
    }

  protected:
    /// Reference bandwidth used by every case below, in resource blocks.
    static constexpr uint32_t REF_RB = 100;
    /// Tx power, matched to the model's reference so that sp = 1.
    static constexpr double TX_DBM = 43.0;
    /// All 14 symbols.
    static constexpr uint16_t ALL14 = 0x3FFF;
    /// Symbols 0..6.
    static constexpr uint16_t FIRST7 = 0x007F;

    /// A gNB PHY configured just far enough for the listener.
    static Ptr<NrGnbPhy> MakePhy(const std::string& pattern, double txPowerDbm = TX_DBM)
    {
        Ptr<NrGnbPhy> phy = CreateObject<NrGnbPhy>();
        phy->SetNumerology(1); // 30 kHz SCS: sets the symbol and slot periods
        phy->SetTxPower(txPowerDbm);
        phy->SetPattern(pattern);
        return phy;
    }

    /// A model whose reference bandwidth and Tx power are pinned.
    static Ptr<NrGnbEnergyModel> MakeModel()
    {
        Ptr<NrGnbEnergyModel> model = CreateObject<NrGnbEnergyModel>();
        model->SetAttribute("ReferenceRbCount", UintegerValue(REF_RB));
        model->SetAttribute("ReferenceTxPowerDbm", DoubleValue(TX_DBM));
        return model;
    }
};

/**
 * @brief A saturated downlink slot gives sf = 1 and exactly P4.
 *
 * TR 38.864 Table 5.1-2 defines Active DL as the P4 state, so a slot occupying
 * the whole carrier in every symbol must report P4 and nothing else. This fails
 * if the listener scales the REGs incorrectly, reports the wrong sf, or never
 * reaches the model at all.
 */
class NrGnbPhyEnergyListenerFullDlTestCase : public NrGnbPhyEnergyListenerSlotTestCase
{
  public:
    NrGnbPhyEnergyListenerFullDlTestCase()
        : NrGnbPhyEnergyListenerSlotTestCase(
              "NrGnbPhyEnergyListener full-DL slot gives sf = 1 and P4")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerFullDlTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> model = MakeModel();
    Ptr<NrGnbPhy> phy = MakePhy("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|");
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    listener->SetEnergyModel(model);
    listener->SetPhy(phy);
    model->Initialize();

    // Every symbol, every RB: the definition of sf = 1.
    listener->SlotEnergyStatsCallback(SfnSf(0, 0, 0, 1), REF_RB, ALL14, REF_RB * 14, 0, 0, 0, 0, 1);

    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastDlSf(), 1.0, 1e-9, "a saturated slot is sf = 1");
    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastSp(), 1.0, 1e-9, "Tx power equals the reference");
    // sa = sf = sp = 1 reduces P_DL to P4 exactly.
    NS_TEST_ASSERT_MSG_EQ_TOL(model->EvaluateCarrierPowerW(),
                              model->CalcDlPowerW(1.0, 1.0, 1.0),
                              1e-9,
                              "a saturated downlink slot must draw exactly P4");
    Simulator::Destroy();
}

/**
 * @brief A mixed slot charges DL, UL and idle symbols at their own power.
 *
 * TR 38.864: "the power consumption in a slot is the sum of the power
 * consumption associated with symbols in the slot." Seven downlink symbols, four
 * uplink and three unallocated must average P_DL, P_UL and P3 in those
 * proportions; one slot-wide power would not.
 */
class NrGnbPhyEnergyListenerMixedSlotTestCase : public NrGnbPhyEnergyListenerSlotTestCase
{
  public:
    NrGnbPhyEnergyListenerMixedSlotTestCase()
        : NrGnbPhyEnergyListenerSlotTestCase(
              "NrGnbPhyEnergyListener classifies DL, UL and idle symbols")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerMixedSlotTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> model = MakeModel();
    Ptr<NrGnbPhy> phy = MakePhy("F|F|F|F|F|F|F|F|F|F|");
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    listener->SetEnergyModel(model);
    listener->SetPhy(phy);
    model->Initialize();

    // Symbols 0..6 downlink at full width, 7..10 uplink, 11..13 unallocated.
    const uint16_t ulMask = 0x0780;
    listener->SlotEnergyStatsCallback(SfnSf(0, 0, 0, 1),
                                      REF_RB,
                                      FIRST7,
                                      REF_RB * 7,
                                      ulMask,
                                      0,
                                      0,
                                      0,
                                      1);

    const double sa = listener->GetLastSa();
    const double pDl = model->CalcDlPowerW(sa, 1.0, listener->GetLastSp());
    const double pUl = model->CalcUlPowerW(sa);
    const double p3 = model->CalcDlPowerW(0.0, 0.0, 0.0); // sa = 0 leaves the P3 baseline

    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastDlSf(),
                              1.0,
                              1e-9,
                              "the downlink symbols occupy the whole carrier");
    NS_TEST_ASSERT_MSG_EQ_TOL(model->EvaluateCarrierPowerW(),
                              (7.0 * pDl + 4.0 * pUl + 3.0 * p3) / 14.0,
                              1e-9,
                              "each symbol must be charged at its own power");
    Simulator::Destroy();
}

/**
 * @brief sp is refreshed per slot from the PHY's live Tx power.
 *
 * sp scales the dynamic part of P_DL, so the listener reads it from the PHY
 * rather than assuming the reference. 10 dB below the reference is a tenth of
 * the power.
 */
class NrGnbPhyEnergyListenerSpTestCase : public NrGnbPhyEnergyListenerSlotTestCase
{
  public:
    NrGnbPhyEnergyListenerSpTestCase()
        : NrGnbPhyEnergyListenerSlotTestCase("NrGnbPhyEnergyListener refreshes sp from the PHY")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerSpTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> model = MakeModel();
    Ptr<NrGnbPhy> phy = MakePhy("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|", TX_DBM - 10.0);
    Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
    listener->SetEnergyModel(model);
    listener->SetPhy(phy);
    model->Initialize();

    listener->SlotEnergyStatsCallback(SfnSf(0, 0, 0, 1), REF_RB, ALL14, REF_RB * 14, 0, 0, 0, 0, 1);

    NS_TEST_ASSERT_MSG_EQ_TOL(listener->GetLastSp(),
                              0.1,
                              1e-9,
                              "10 dB below the reference must give sp = 0.1");
    NS_TEST_ASSERT_MSG_LT(model->EvaluateCarrierPowerW(),
                          model->CalcDlPowerW(1.0, 1.0, 1.0),
                          "and a reduced sp must draw less than P4");
    Simulator::Destroy();
}

/**
 * @brief Downlink capability is read off the PHY's TDD pattern.
 *
 * The model uses this to keep an uplink-only bandwidth part out of the carrier's
 * downlink reference bandwidth; that behaviour is covered at the model level,
 * but nothing checked that the listener derives the flag correctly, and every
 * other case here uses a downlink-capable pattern.
 */
class NrGnbPhyEnergyListenerDlCapableTestCase : public NrGnbPhyEnergyListenerSlotTestCase
{
  public:
    NrGnbPhyEnergyListenerDlCapableTestCase()
        : NrGnbPhyEnergyListenerSlotTestCase(
              "NrGnbPhyEnergyListener derives DL capability from the pattern")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerDlCapableTestCase::DoRun()
{
    struct Case
    {
        const char* pattern;
        bool dlCapable;
        const char* why;
    };

    const Case cases[] = {
        {"DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|", true, "an all-DL pattern"},
        {"F|F|F|F|F|F|F|F|F|F|", true, "a flexible pattern is dually schedulable"},
        {"UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|", false, "an uplink-only pattern"},
    };

    for (const auto& c : cases)
    {
        Ptr<NrGnbEnergyModel> model = MakeModel();
        Ptr<NrGnbPhyEnergyListener> listener = CreateObject<NrGnbPhyEnergyListener>();
        listener->SetEnergyModel(model);
        listener->SetPhy(MakePhy(c.pattern));

        NS_TEST_ASSERT_MSG_EQ(listener->IsDlCapable(), c.dlCapable, c.why);
    }
    Simulator::Destroy();
}

/**
 * @brief sp is a carrier property, summed over the bandwidth parts transmitting.
 *
 * Tx power is configured per bandwidth part, but TR 38.864 scales by the power of
 * the DL transmission as a whole against the reference configuration. Two parts
 * of one carrier at half power each therefore transmit at the reference, not at
 * half of it. Pushing each part's power straight into the model instead would
 * leave the carrier's sp at whichever part reported last - the same mistake sf
 * used to make with the reporting part's bandwidth.
 */
class NrGnbPhyEnergyListenerCarrierSpTestCase : public NrGnbPhyEnergyListenerSlotTestCase
{
  public:
    NrGnbPhyEnergyListenerCarrierSpTestCase()
        : NrGnbPhyEnergyListenerSlotTestCase(
              "NrGnbPhyEnergyListener sums Tx power across the carrier's BWPs for sp")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbPhyEnergyListenerCarrierSpTestCase::DoRun()
{
    // Two bandwidth parts of ONE carrier, each at exactly half the reference
    // power, so together they transmit at exactly the reference. Derived rather
    // than written as "-3 dB", which is only half power to four decimal places.
    const double halfPowerDbm = TX_DBM + 10.0 * std::log10(0.5);

    Ptr<NrGnbEnergyModel> model = MakeModel();
    Ptr<NrGnbPhyEnergyListener> l0 = CreateObject<NrGnbPhyEnergyListener>();
    Ptr<NrGnbPhyEnergyListener> l1 = CreateObject<NrGnbPhyEnergyListener>();
    l0->SetEnergyModel(model);
    l0->SetPhy(MakePhy("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|", halfPowerDbm));
    l1->SetEnergyModel(model);
    l1->SetPhy(MakePhy("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|", halfPowerDbm));
    model->Initialize();

    // Each part saturates half the carrier, so together sf = 1 as well.
    l0->SlotEnergyStatsCallback(SfnSf(0, 0, 0, 1), REF_RB, ALL14, (REF_RB / 2) * 14, 0, 0, 0, 0, 1);
    l1->SlotEnergyStatsCallback(SfnSf(0, 0, 0, 1), REF_RB, ALL14, (REF_RB / 2) * 14, 0, 0, 0, 1, 1);

    // sa = sf = sp = 1 reduces P_DL to P4. Taking one part's power alone would
    // give sp = 0.5 and land near 212.5 W instead.
    NS_TEST_ASSERT_MSG_EQ_TOL(model->EvaluateCarrierPowerW(),
                              model->CalcDlPowerW(1.0, 1.0, 1.0),
                              1e-6,
                              "two half-power BWPs of one carrier must together reach P4");
    Simulator::Destroy();
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
        AddTestCase(new NrGnbPhyEnergyListenerFullDlTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerMixedSlotTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerSpTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerDlCapableTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbPhyEnergyListenerCarrierSpTestCase(), Duration::QUICK);
    }
};

static NrGnbPhyEnergyListenerTestSuite g_nrGnbPhyEnergyListenerTestSuite; //!< the test suite

} // namespace ns3
