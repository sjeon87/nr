// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/basic-energy-source.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/nr-gnb-energy-aggregator.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-ue-drx-model.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <string>
#include <vector>

/**
 * @file nr-test-energy-models.cc
 * @ingroup test
 *
 * @brief Unit tests for NrUeEnergyModel (TR 38.840), NrGnbEnergyModel
 * (TR 38.864) and NrUeDrxModel: the pure spec-derived math and the
 * time-dependent energy accounting / DRX cycling.
 */
namespace ns3
{

/**
 * @brief TR 38.840 Table 18 power tables and Section 8.1.3 scalings.
 */
class NrUeEnergyModelPowerTestCase : public TestCase
{
  public:
    NrUeEnergyModelPowerTestCase()
        : TestCase("NrUeEnergyModel TR 38.840 power table and scalings")
    {
    }

  private:
    void DoRun() override;
};

void
NrUeEnergyModelPowerTestCase::DoRun()
{
    Ptr<NrUeEnergyModel> ue = CreateObject<NrUeEnergyModel>();

    // TR 38.840 Table 18 FR1 relative powers (Deep Sleep = 1).
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_DEEP_SLEEP), 1.0, 1e-9, "deep sleep");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_LIGHT_SLEEP), 20.0, 1e-9, "light sleep");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_MICRO_SLEEP), 45.0, 1e-9, "micro sleep");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_PDCCH_ONLY), 100.0, 1e-9, "PDCCH-only");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_SSB_CSI_RS), 100.0, 1e-9, "SSB/CSI-RS");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_PDCCH_PDSCH), 300.0, 1e-9, "PDCCH+PDSCH");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_UL_TX), 250.0, 1e-9, "UL 0 dBm anchor");

    // UL power interpolates linearly between the 0 dBm and 23 dBm anchors.
    ue->SetUlTxPowerDbm(23.0);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_UL_TX), 700.0, 1e-9, "UL at 23 dBm");
    ue->SetUlTxPowerDbm(11.5);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_UL_TX), 475.0, 1e-9, "UL at 11.5 dBm");
    ue->SetUlTxPowerDbm(0.0);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetRelativePower(NR_UE_UL_TX), 250.0, 1e-9, "UL back at 0 dBm");

    // BWP scaling: scale(X) = 0.4 + 0.6 * (X - 20) / 80 (TR 38.840 8.1.3).
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->ScaleBwp(100), 1.0, 1e-9, "100 MHz reference");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->ScaleBwp(40), 0.55, 1e-9, "40 MHz");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->ScaleBwp(20), 0.4, 1e-9, "20 MHz");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->ScaleBwp(10), 0.325, 1e-9, "10 MHz");

    // BWP scaling applies to the active DL states only (PowerUnit = 1 mW).
    ue->ApplyBwpScaling(20);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_PDCCH_PDSCH),
                              0.12,
                              1e-9,
                              "300 units x 0.4 x 1 mW");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_DEEP_SLEEP),
                              1e-3,
                              1e-12,
                              "sleep states are unscaled");
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_UL_TX), 0.25, 1e-9, "UL is unscaled by BWP");

    // Antenna scaling: each halving from the 4-antenna reference is one 0.7x.
    ue->ApplyBwpScaling(100);
    ue->ApplyAntennaScaling(2);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_PDCCH_ONLY),
                              0.07,
                              1e-9,
                              "P_2Rx = 0.7 x P_4Rx");
    ue->ApplyAntennaScaling(4);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_PDCCH_ONLY),
                              0.1,
                              1e-9,
                              "reference antennas restore full power");

    // Blind-decoding reduction: multiplier = alpha + (1 - alpha) * 0.7.
    ue->ApplyBdReduction(0.5);
    NS_TEST_ASSERT_MSG_EQ_TOL(ue->GetStatePowerW(NR_UE_PDCCH_ONLY),
                              0.085,
                              1e-9,
                              "alpha = 0.5 -> 0.85 multiplier");

    // FR2 table.
    Ptr<NrUeEnergyModel> fr2 = CreateObject<NrUeEnergyModel>();
    fr2->SetAttribute("FreqRange", EnumValue(NrUeEnergyModel::FR2));
    NS_TEST_ASSERT_MSG_EQ_TOL(fr2->GetRelativePower(NR_UE_PDCCH_ONLY), 175.0, 1e-9, "FR2 PDCCH");
    NS_TEST_ASSERT_MSG_EQ_TOL(fr2->GetRelativePower(NR_UE_MICRO_SLEEP), 45.0, 1e-9, "FR2 micro");
    fr2->SetUlTxPowerDbm(5.0);
    NS_TEST_ASSERT_MSG_EQ_TOL(fr2->GetRelativePower(NR_UE_UL_TX),
                              350.0,
                              1e-9,
                              "FR2 UL is level-independent");
}

/**
 * @brief TR 38.840 Table 21 receive-chain scaling from an explicit configuration.
 *
 * Halving the powered chains relative to the reference applies one 0.7 factor,
 * so FR1 with 2 of the reference 4 chains draws 0.7 of the 4Rx power.
 */
class NrUeEnergyModelRxChainScalingTestCase : public TestCase
{
  public:
    NrUeEnergyModelRxChainScalingTestCase()
        : TestCase("NrUeEnergyModel Table 21 scaling from ActiveRxChains")
    {
    }

  private:
    void DoRun() override;
};

void
NrUeEnergyModelRxChainScalingTestCase::DoRun()
{
    // Unset ActiveRxChains means all reference chains powered -> no scaling.
    Ptr<NrUeEnergyModel> full = CreateObject<NrUeEnergyModel>();
    full->Initialize();
    double p4Rx = full->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    Ptr<NrUeEnergyModel> half = CreateObject<NrUeEnergyModel>();
    half->SetAttribute("ActiveRxChains", UintegerValue(2));
    half->Initialize();
    double p2Rx = half->GetStatePowerW(NR_UE_PDCCH_PDSCH);

    NS_TEST_ASSERT_MSG_GT(p4Rx, 0.0, "reference active power must be positive");
    NS_TEST_ASSERT_MSG_EQ_TOL(p2Rx / p4Rx, 0.7, 1e-9, "FR1 P_2Rx must be 0.7 * P_4Rx");
}

/**
 * @brief A transition transient costs extraPower x duration, once.
 *
 * The transient rides on top of the state power, so every power query inside its
 * window includes it. That must not turn into extra energy: interleaved state
 * changes only split the interval, they do not re-charge it. Re-triggering while
 * one is in flight extends the window rather than being cut short by the pending
 * end event.
 */
/**
 * @brief TR 38.840 Table 19 sleep transition energy, by value.
 *
 * Entering deep or light sleep costs an additional transition energy on top of
 * the sleep power itself. Nothing asserted those two numbers, so a wrong or
 * missing row was invisible: the model reports zero for any state with no
 * tabulated transition, which is correct for micro sleep and every awake state
 * and silently wrong for these two.
 *
 * The expectation is built from public accessors plus the one Table 19 constant
 * under test, so it pins that constant and nothing else.
 */
class NrUeEnergyModelTable19TestCase : public TestCase
{
  public:
    NrUeEnergyModelTable19TestCase()
        : TestCase("NrUeEnergyModel TR 38.840 Table 19 sleep transition energy")
    {
    }

  private:
    void DoRun() override;
    /// Energy [J] of: awake until t1, then @p sleep until t2.
    static double Run(NrUePowerState sleep, Time t1, Time t2);
};

double
NrUeEnergyModelTable19TestCase::Run(NrUePowerState sleep, Time t1, Time t2)
{
    Ptr<NrUeEnergyModel> ue = CreateObject<NrUeEnergyModel>();
    ue->ChangeState(NR_UE_PDCCH_ONLY);
    Simulator::Schedule(t1, [ue, sleep]() { ue->ChangeState(sleep); });
    Simulator::Stop(t2);
    Simulator::Run();
    const double energyJ = ue->GetTotalEnergyJ();
    Simulator::Destroy();
    return energyJ;
}

void
NrUeEnergyModelTable19TestCase::DoRun()
{
    const Time t1 = MilliSeconds(100);
    const Time t2 = MilliSeconds(300); // well past the 20 ms deep-sleep transition

    Ptr<NrUeEnergyModel> ref = CreateObject<NrUeEnergyModel>();
    const double pAwake = ref->GetStatePowerW(NR_UE_PDCCH_ONLY);

    // PowerUnit defaults to 1 mW, and Table 19 energy is [relative power x ms],
    // so E[J] = relative x 1e-3 W x 1e-3 s.
    struct Expected
    {
        NrUePowerState state;
        double energyRelMs;
        const char* name;
    };

    const Expected rows[] = {
        {NR_UE_DEEP_SLEEP, 450.0, "deep sleep"},
        {NR_UE_LIGHT_SLEEP, 100.0, "light sleep"},
        {NR_UE_MICRO_SLEEP, 0.0, "micro sleep (immediate, no transition)"},
    };

    for (const auto& r : rows)
    {
        const double pSleep = ref->GetStatePowerW(r.state);
        const double stateJ = pAwake * t1.GetSeconds() + pSleep * (t2 - t1).GetSeconds();
        const double transitionJ = r.energyRelMs * 1e-6;

        NS_TEST_ASSERT_MSG_EQ_TOL(Run(r.state, t1, t2),
                                  stateJ + transitionJ,
                                  1e-12,
                                  "Table 19 transition energy for " << r.name);
    }
}

class NrUeEnergyModelTransientTestCase : public TestCase
{
  public:
    NrUeEnergyModelTransientTestCase()
        : TestCase("NrUeEnergyModel transition transient charges exactly once")
    {
    }

  private:
    void DoRun() override;
    /// Run one scenario and return the total energy [J].
    double Run(uint32_t nChanges, bool transient, bool reTrigger);
};

double
NrUeEnergyModelTransientTestCase::Run(uint32_t nChanges, bool transient, bool reTrigger)
{
    Ptr<NrUeEnergyModel> ue = CreateObject<NrUeEnergyModel>();
    if (transient)
    {
        ue->SetAttribute("SetupTransitionPower", DoubleValue(1.0)); // 1 W extra
        ue->SetAttribute("SetupTransitionTime", TimeValue(MilliSeconds(5)));
    }
    Simulator::Schedule(MilliSeconds(100), [ue, transient]() {
        if (transient)
        {
            ue->TriggerSetupTransition();
        }
        ue->ChangeState(NR_UE_PDCCH_PDSCH);
    });
    // Redundant state changes across and past the transient window. The last of
    // these lands exactly on the transition end time, which is the case that used
    // to lose the final sliver of transient energy.
    for (uint32_t i = 1; i <= nChanges; ++i)
    {
        Simulator::Schedule(MilliSeconds(100) + MicroSeconds(100 * i),
                            [ue]() { ue->ChangeState(NR_UE_PDCCH_PDSCH); });
    }
    if (reTrigger)
    {
        Simulator::Schedule(MilliSeconds(102), [ue]() { ue->TriggerSetupTransition(); });
    }
    Simulator::Stop(MilliSeconds(200));
    Simulator::Run();
    double energyJ = ue->GetTotalEnergyJ();
    Simulator::Destroy();
    return energyJ;
}

void
NrUeEnergyModelTransientTestCase::DoRun()
{
    double base = Run(0, false, false);

    // 1 W for 5 ms = 5 mJ on top of the state energy, however the interval is cut.
    NS_TEST_ASSERT_MSG_EQ_TOL(Run(0, true, false) - base,
                              0.005,
                              1e-12,
                              "transient alone must cost extraPower x duration");
    NS_TEST_ASSERT_MSG_EQ_TOL(Run(60, true, false) - base,
                              0.005,
                              1e-12,
                              "interleaved state changes must not change the transient cost");

    // Re-triggering at 102 ms extends the window to 107 ms: 7 ms in total.
    NS_TEST_ASSERT_MSG_EQ_TOL(Run(0, true, true) - base,
                              0.007,
                              1e-12,
                              "re-triggering must extend the transient, not truncate it");

    // Interleaved state changes must not change the no-transient baseline either.
    NS_TEST_ASSERT_MSG_EQ_TOL(Run(60, false, false),
                              base,
                              1e-12,
                              "state changes alone must not create energy");
}

/**
 * @brief State-change energy integration and occupancy accounting.
 */
class NrUeEnergyModelAccountingTestCase : public TestCase
{
  public:
    NrUeEnergyModelAccountingTestCase()
        : TestCase("NrUeEnergyModel energy integration and occupancy")
    {
    }

  private:
    void DoRun() override;

    Ptr<NrUeEnergyModel> m_ue; //!< Model under test
};

void
NrUeEnergyModelAccountingTestCase::DoRun()
{
    m_ue = CreateObject<NrUeEnergyModel>();

    // [0,1]s deep sleep (1 unit), [1,2]s PDCCH-only (100 units), [2,3]s deep
    // sleep again. PowerUnit = 1 mW per unit.
    Simulator::Schedule(Seconds(1),
                        &NrUeEnergyModel::ChangeState,
                        m_ue,
                        static_cast<int>(NR_UE_PDCCH_ONLY));
    Simulator::Schedule(Seconds(2),
                        &NrUeEnergyModel::ChangeState,
                        m_ue,
                        static_cast<int>(NR_UE_DEEP_SLEEP));
    Simulator::Stop(Seconds(3));
    Simulator::Run();

    // State energy: 1e-3*1 + 0.1*1 + 1e-3*1 = 0.102 J (open interval included),
    // plus the TR 38.840 Table 19 deep-sleep transition charged on entering deep
    // sleep at t=2: 450 [relative power x ms] x 1 mW/unit = 4.5e-4 J.
    NS_TEST_ASSERT_MSG_EQ_TOL(m_ue->GetTotalEnergyJ(), 0.10245, 1e-9, "integrated energy");
    NS_TEST_ASSERT_MSG_EQ_TOL(m_ue->GetStateTimeFraction(NR_UE_DEEP_SLEEP),
                              2.0 / 3.0,
                              1e-9,
                              "deep sleep occupancy");
    NS_TEST_ASSERT_MSG_EQ_TOL(m_ue->GetStateTimeFraction(NR_UE_PDCCH_ONLY),
                              1.0 / 3.0,
                              1e-9,
                              "PDCCH-only occupancy");
    // Average relative power = 2/3 * 1 + 1/3 * 100 = 34 power-units.
    NS_TEST_ASSERT_MSG_EQ_TOL(m_ue->GetAverageRelativePower(), 34.0, 1e-9, "average power");

    Simulator::Destroy();
}

/**
 * @brief Full TR 38.864 Table 5.1-3 coverage over both table axes.
 *
 * BS category and reference configuration set are independent axes and every
 * combination is tabulated, so all six rows must be distinct and ordered
 * P1 < P2 < P3 < P5 < P4. Deep and light sleep are merged cells in the table:
 * one value per category, shared by all three sets.
 */
class NrGnbEnergyModelPowerTableTestCase : public TestCase
{
  public:
    NrGnbEnergyModelPowerTableTestCase()
        : TestCase("NrGnbEnergyModel TR 38.864 Table 5.1-3 for every category and set")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelPowerTableTestCase::DoRun()
{
    // TR 38.864 Table 5.1-3, indexed [category][set] as {P1, P2, P3, P4, P5}.
    const double table[2][3][5] = {
        {{1.0, 25.0, 55.0, 280.0, 110.0},
         {1.0, 25.0, 50.0, 200.0, 90.0},
         {1.0, 25.0, 38.0, 152.0, 80.0}},
        {{1.0, 2.1, 5.5, 32.0, 6.5}, {1.0, 2.1, 5.0, 26.0, 5.8}, {1.0, 2.1, 3.0, 17.6, 4.2}},
    };
    const std::string cats[2] = {"BsCat1", "BsCat2"};
    const std::string sets[3] = {"Set1", "Set2", "Set3"};

    for (uint32_t c = 0; c < 2; ++c)
    {
        for (uint32_t s = 0; s < 3; ++s)
        {
            Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
            gnb->SetAttribute("BsCategory", StringValue(cats[c]));
            gnb->SetAttribute("RefConfigSet", StringValue(sets[s]));
            std::string ctx = cats[c] + "/" + sets[s];

            double p1 = gnb->GetRelativePower(NrGnbPowerState::DeepSleep);
            double p2 = gnb->GetRelativePower(NrGnbPowerState::LightSleep);
            double p3 = gnb->GetRelativePower(NrGnbPowerState::MicroSleep);
            double p4 = gnb->GetRelativePower(NrGnbPowerState::ActiveDl);
            double p5 = gnb->GetRelativePower(NrGnbPowerState::ActiveUl);

            NS_TEST_ASSERT_MSG_EQ_TOL(p1, table[c][s][0], 1e-9, "P1 " << ctx);
            NS_TEST_ASSERT_MSG_EQ_TOL(p2, table[c][s][1], 1e-9, "P2 " << ctx);
            NS_TEST_ASSERT_MSG_EQ_TOL(p3, table[c][s][2], 1e-9, "P3 " << ctx);
            NS_TEST_ASSERT_MSG_EQ_TOL(p4, table[c][s][3], 1e-9, "P4 " << ctx);
            NS_TEST_ASSERT_MSG_EQ_TOL(p5, table[c][s][4], 1e-9, "P5 " << ctx);

            // Sleep depths and the DL/UL split must stay ordered in every row.
            NS_TEST_ASSERT_MSG_LT(p1, p2, "P1 must be below P2 for " << ctx);
            NS_TEST_ASSERT_MSG_LT(p2, p3, "P2 must be below P3 for " << ctx);
            NS_TEST_ASSERT_MSG_LT(p3, p5, "P3 must be below P5 for " << ctx);
            NS_TEST_ASSERT_MSG_LT(p5, p4, "P5 must be below P4 for " << ctx);
        }
    }
}

/**
 * @brief A named TR 38.864 Table 5.1-1 set fixes the reference Tx power.
 *
 * The set is authoritative for the reference power, which nothing in the
 * scenario supplies; Custom (the default) leaves it as configured, so existing
 * configurations keep their meaning.
 */
class NrGnbEnergyModelRefConfigBundleTestCase : public TestCase
{
  public:
    NrGnbEnergyModelRefConfigBundleTestCase()
        : TestCase("NrGnbEnergyModel named reference set fixes the reference Tx power")
    {
    }

  private:
    void DoRun() override;
    /// sp at a given actual Tx power, for the model under test.
    double SpAt(Ptr<NrGnbEnergyModel> gnb, double txDbm);
};

double
NrGnbEnergyModelRefConfigBundleTestCase::SpAt(Ptr<NrGnbEnergyModel> gnb, double txDbm)
{
    gnb->SetTxPowerDbm(txDbm);
    return gnb->GetSp();
}

void
NrGnbEnergyModelRefConfigBundleTestCase::DoRun()
{
    // Set 3 is the FR2 small cell: 33 dBm reference. A gNB transmitting at its
    // own reference power must report sp = 1, not the ~0.006 it would get if the
    // reference were left at the Set 1 macro value of 55 dBm.
    Ptr<NrGnbEnergyModel> set3 = CreateObject<NrGnbEnergyModel>();
    set3->SetAttribute("RefConfigSet", StringValue("Set3"));
    set3->Initialize();
    NS_TEST_ASSERT_MSG_EQ_TOL(SpAt(set3, 33.0), 1.0, 1e-9, "Set 3 reference must be 33 dBm");

    Ptr<NrGnbEnergyModel> set2 = CreateObject<NrGnbEnergyModel>();
    set2->SetAttribute("RefConfigSet", StringValue("Set2"));
    set2->Initialize();
    NS_TEST_ASSERT_MSG_EQ_TOL(SpAt(set2, 49.0), 1.0, 1e-9, "Set 2 reference must be 49 dBm");

    Ptr<NrGnbEnergyModel> set1 = CreateObject<NrGnbEnergyModel>();
    set1->SetAttribute("RefConfigSet", StringValue("Set1"));
    set1->Initialize();
    NS_TEST_ASSERT_MSG_EQ_TOL(SpAt(set1, 55.0), 1.0, 1e-9, "Set 1 reference must be 55 dBm");

    // Custom is the default and must not touch a user-supplied reference.
    Ptr<NrGnbEnergyModel> custom = CreateObject<NrGnbEnergyModel>();
    custom->SetAttribute("ReferenceTxPowerDbm", DoubleValue(40.0));
    custom->Initialize();
    NS_TEST_ASSERT_MSG_EQ_TOL(SpAt(custom, 40.0), 1.0, 1e-9, "Custom must keep the set value");

    // Custom must also keep the pre-existing Set 1 power row, so that defaulting
    // to Custom does not change what an existing configuration computes.
    NS_TEST_ASSERT_MSG_EQ_TOL(custom->GetRelativePower(NrGnbPowerState::ActiveDl),
                              280.0,
                              1e-9,
                              "Custom must keep the Set 1 power row");
}

/**
 * @brief TR 38.864 Table 5.1-4 / 5.1-5 sleep transitions, and the Guard state.
 *
 * The transition is an energy and a time charged when a sleep state is entered,
 * not a power level. Guard is the DL/UL turnaround and is unrelated to it.
 */
class NrGnbEnergyModelTransitionTestCase : public TestCase
{
  public:
    NrGnbEnergyModelTransitionTestCase()
        : TestCase("NrGnbEnergyModel TR 38.864 sleep transition energy and Guard state")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelTransitionTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> cat1 = CreateObject<NrGnbEnergyModel>();

    // Table 5.1-5, BS Category 1, PowerUnit = 1 W: energy is [relative x ms].
    NS_TEST_ASSERT_MSG_EQ_TOL(cat1->GetTransitionEnergyJ(NrGnbPowerState::DeepSleep),
                              1.0,
                              1e-12,
                              "Cat 1 deep sleep: 1000 x 1 W x 1 ms");
    NS_TEST_ASSERT_MSG_EQ_TOL(cat1->GetTransitionEnergyJ(NrGnbPowerState::LightSleep),
                              0.09,
                              1e-12,
                              "Cat 1 light sleep: 90 x 1 W x 1 ms");
    NS_TEST_ASSERT_MSG_EQ_TOL(cat1->GetTransitionEnergyJ(NrGnbPowerState::MicroSleep),
                              0.0,
                              1e-12,
                              "micro sleep is immediate");
    // Table 5.1-4 total transition times.
    NS_TEST_ASSERT_MSG_EQ(cat1->GetTransitionTime(NrGnbPowerState::DeepSleep),
                          MilliSeconds(50),
                          "Cat 1 deep sleep transition time");
    NS_TEST_ASSERT_MSG_EQ(cat1->GetTransitionTime(NrGnbPowerState::LightSleep),
                          MilliSeconds(6),
                          "Cat 1 light sleep transition time");

    Ptr<NrGnbEnergyModel> cat2 = CreateObject<NrGnbEnergyModel>();
    cat2->SetAttribute("BsCategory", StringValue("BsCat2"));
    NS_TEST_ASSERT_MSG_EQ_TOL(cat2->GetTransitionEnergyJ(NrGnbPowerState::DeepSleep),
                              17.0,
                              1e-12,
                              "Cat 2 deep sleep: 17000 x 1 W x 1 ms");
    NS_TEST_ASSERT_MSG_EQ(cat2->GetTransitionTime(NrGnbPowerState::DeepSleep),
                          Seconds(10),
                          "Cat 2 deep sleep transition time");

    // Guard is the DL/UL turnaround: charged at P3, and it is not a sleep state,
    // so entering it must not charge any transition energy.
    NS_TEST_ASSERT_MSG_EQ_TOL(cat1->GetRelativePower(NrGnbPowerState::Guard),
                              cat1->GetRelativePower(NrGnbPowerState::MicroSleep),
                              1e-12,
                              "Guard is charged at P3");
    NS_TEST_ASSERT_MSG_EQ_TOL(cat1->GetTransitionEnergyJ(NrGnbPowerState::Guard),
                              0.0,
                              1e-12,
                              "Guard is not a 3GPP sleep transition");

    // Entering deep sleep from active charges the Table 5.1-5 energy E once. It is
    // not a lump: TR 38.864 Section 5.1 says the sleep power is drawn throughout
    // the transition and E is *additional* over the total transition time T, so
    // the transition window costs P_sleep*T + E. Spreading it as an E/T transient
    // is also what makes it visible to an attached EnergySource, which can only
    // integrate the current reported by DoGetCurrentA().
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->Initialize();
    const Time transitionTime = gnb->GetTransitionTime(NrGnbPowerState::DeepSleep);
    const double transitionEnergyJ = gnb->GetTransitionEnergyJ(NrGnbPowerState::DeepSleep);
    const Time postWake = MilliSeconds(20);
    double before = 0.0;
    double afterTransition = 0.0;
    double afterWake = 0.0;
    double sleepPowerW = 0.0;

    Simulator::Schedule(MilliSeconds(100), [&]() {
        before = gnb->GetTotalEnergyJ();
        gnb->ChangeState(static_cast<int>(NrGnbPowerState::DeepSleep));
        sleepPowerW = gnb->GetCurrentPowerW(); // state power, without the transient
    });
    Simulator::Schedule(MilliSeconds(100) + transitionTime,
                        [&]() { afterTransition = gnb->GetTotalEnergyJ(); });
    Simulator::Schedule(MilliSeconds(100) + transitionTime + postWake, [&]() {
        afterWake = gnb->GetTotalEnergyJ();
        gnb->ChangeState(static_cast<int>(NrGnbPowerState::ActiveDl));
    });
    Simulator::Stop(MilliSeconds(100) + transitionTime + postWake + MilliSeconds(1));
    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ_TOL(afterTransition - before,
                              sleepPowerW * transitionTime.GetSeconds() + transitionEnergyJ,
                              1e-12,
                              "the transition window costs P_sleep*T plus the Table 5.1-5 energy");
    // Past the transient, only the plain sleep power is drawn: E is charged once,
    // and waking does not charge it again since the tabulated value covers both
    // ramp directions.
    NS_TEST_ASSERT_MSG_EQ_TOL(afterWake - afterTransition,
                              sleepPowerW * postWake.GetSeconds(),
                              1e-12,
                              "after the transition only the sleep power is drawn");
    Simulator::Destroy();
}

/**
 * @brief TR 38.864 Section 5.1 sf normalisation.
 *
 * sf is "the ratio between the RF bandwidth and the maximum system BW", so the
 * denominator is the carrier's reference bandwidth and NOT the bandwidth of the BWP
 * reporting the allocation. If it were the reporting BWP, numerator and denominator
 * would shrink together and a fully loaded BWP would always give sf = 1 however
 * narrow it is -- which would make BWP adaptation (Section 6.2.2) show no saving.
 */
class NrGnbEnergyModelSfNormalizationTestCase : public TestCase
{
  public:
    NrGnbEnergyModelSfNormalizationTestCase()
        : TestCase("NrGnbEnergyModel sf is normalised to the reference bandwidth")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelSfNormalizationTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->SetAttribute("ReferenceRbCount", UintegerValue(100)); // whole carrier = 100 RBs

    // A 50-RB BWP, fully allocated over 14 symbols, occupies half the carrier.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcSf(50 * 14, 14, 50),
                              0.5,
                              1e-12,
                              "a saturated half-width BWP must give sf = 0.5, not 1.0");
    // The same BWP spanning the whole carrier saturates sf.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcSf(100 * 14, 14, 100),
                              1.0,
                              1e-12,
                              "a saturated full-width BWP must give sf = 1");
    // Partial load scales linearly, and the reporting BWP's own width is irrelevant.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcSf(25 * 14, 14, 50),
                              0.25,
                              1e-12,
                              "sf must scale with occupancy of the carrier, not of the BWP");

    // sf = 1 must reproduce P4 exactly, which is what makes "Active DL" the P4 state
    // of Table 5.1-2. This is the check that settles what belongs in the denominator.
    double p4W = gnb->GetRelativePower(NrGnbPowerState::ActiveDl);
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcDlPowerW(1.0, 1.0, 1.0),
                              p4W,
                              1e-12,
                              "sa = sf = sp = 1 must yield exactly P4");

    // Left unset, the reference falls back to the reporting BWP (legacy behaviour).
    Ptr<NrGnbEnergyModel> unset = CreateObject<NrGnbEnergyModel>();
    NS_TEST_ASSERT_MSG_EQ_TOL(unset->CalcSf(50 * 14, 14, 50),
                              1.0,
                              1e-12,
                              "with no reference set, the reporting BWP is the denominator");

    Simulator::Destroy();
}

/**
 * @brief Aggregating the bandwidth parts of one carrier (TR 38.864 Section 5.1).
 *
 * P_DL contains the static baseline P3 and the load-independent share A, so
 * evaluating it per BWP and then summing or averaging counts both once per BWP.
 * The occupancies must be aggregated first and the formula evaluated once for
 * the carrier: REGs add, symbol counts take the union, and the sf denominator is
 * the whole carrier.
 */
class NrGnbEnergyModelCarrierAggregationTestCase : public TestCase
{
  public:
    NrGnbEnergyModelCarrierAggregationTestCase()
        : TestCase("NrGnbEnergyModel aggregates BWP occupancy before applying the formula")
    {
    }

  private:
    void DoRun() override;

    /// Build one BWP's slot record. 100 RBs, 14 symbols, DL on the given symbols.
    static NrGnbEnergyModel::BwpOccupancy Bwp(uint16_t dlMask, uint32_t dlReg)
    {
        NrGnbEnergyModel::BwpOccupancy o;
        o.rbCount = 100;
        o.dlDataMask = dlMask;
        o.dlDataReg = dlReg;
        o.symbolsPerSlot = 14;
        return o;
    }

    static constexpr uint16_t ALL14 = 0x3FFF;  //!< symbols 0..13
    static constexpr uint16_t FIRST7 = 0x007F; //!< symbols 0..6
    static constexpr uint16_t LAST7 = 0x3F80;  //!< symbols 7..13
};

void
NrGnbEnergyModelCarrierAggregationTestCase::DoRun()
{
    // Cat 1 / Set 1 defaults: P3 = 55, P4 = 280, A = 0.4, sa = sp = eta = 1.
    // Two 100-RB BWPs, so the carrier is 200 RBs and is derived from them.
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->Initialize();

    // Both idle: the carrier draws P3 ONCE, not once per BWP.
    gnb->ReportBwpOccupancy(0, Bwp(0, 0));
    gnb->ReportBwpOccupancy(1, Bwp(0, 0));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              55.0,
                              1e-9,
                              "two idle BWPs are one idle carrier: P3 once, not 2 x P3");

    // BWP 0 saturated, BWP 1 silent. Half the carrier is occupied, so sf = 0.5.
    // The symbol count is the UNION (14, not 0 + 14), and the REGs add.
    gnb->ReportBwpOccupancy(0, Bwp(ALL14, 100 * 14));
    gnb->ReportBwpOccupancy(1, Bwp(0, 0));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              212.5, // 55 + 225 * (0.4 + 0.5 * 0.6)
                              1e-9,
                              "half-occupied carrier must evaluate P_DL once at sf = 0.5");

    // The two ways of combining per-BWP powers both give something else, which is
    // the whole reason the aggregation happens before the formula:
    //   summing   -> 280 + 55  = 335
    //   averaging -> (280 + 55) / 2 = 167.5
    NS_TEST_ASSERT_MSG_GT(std::abs(gnb->EvaluateCarrierPowerW() - 335.0),
                          1.0,
                          "must not be the SUM of the per-BWP powers");
    NS_TEST_ASSERT_MSG_GT(std::abs(gnb->EvaluateCarrierPowerW() - 167.5),
                          1.0,
                          "must not be the AVERAGE of the per-BWP powers");

    // Both saturated: the carrier is full, so sf = 1 and the power is exactly P4.
    gnb->ReportBwpOccupancy(0, Bwp(ALL14, 100 * 14));
    gnb->ReportBwpOccupancy(1, Bwp(ALL14, 100 * 14));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              280.0,
                              1e-9,
                              "a fully occupied carrier must reach exactly P4");

    // Partial overlap: BWP 0 uses all 14 symbols, BWP 1 only 7. The union is 14
    // symbols and sf is the mean occupancy over them, 0.75. That equals the true
    // per-symbol answer ((1.0 x 7 + 0.5 x 7) / 14) because P_DL is affine in sf.
    gnb->ReportBwpOccupancy(0, Bwp(ALL14, 100 * 14));
    gnb->ReportBwpOccupancy(1, Bwp(FIRST7, 100 * 7));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              246.25, // 7 symbols at sf 1.0, 7 at sf 0.5
                              1e-9,
                              "nested spans: 7 symbols fully occupied, 7 half occupied");

    // DISJOINT spans. The counts here are identical to a case where the two BWPs
    // overlap - 7 active symbols each, 700 REGs each - so counts alone cannot tell
    // the two apart. Positions can: every symbol has exactly one BWP active at
    // 100 of the carrier's 200 RBs, so sf = 0.5 throughout and the carrier draws
    // 212.5 W. Combining by symbol COUNT gave 167.50 W here, a 21% under-count.
    gnb->ReportBwpOccupancy(0, Bwp(FIRST7, 100 * 7));
    gnb->ReportBwpOccupancy(1, Bwp(LAST7, 100 * 7));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              212.5,
                              1e-9,
                              "disjoint spans: every symbol half occupied, so sf = 0.5 "
                              "throughout - counts cannot distinguish this from overlap");

    // A BWP that stops reporting must stop contributing rather than linger.
    Ptr<NrGnbEnergyModel> stale = CreateObject<NrGnbEnergyModel>();
    stale->Initialize();
    auto expiring = Bwp(ALL14, 100 * 14);
    expiring.validUntil = MicroSeconds(1); // 0 would mean "never expires"
    stale->ReportBwpOccupancy(0, expiring);
    stale->ReportBwpOccupancy(1, Bwp(0, 0));
    Simulator::Schedule(MilliSeconds(1), [stale, this]() {
        NS_TEST_ASSERT_MSG_EQ_TOL(stale->EvaluateCarrierPowerW(),
                                  55.0,
                                  1e-9,
                                  "an expired BWP record must not keep contributing");
    });
    Simulator::Stop(MilliSeconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @brief A carrier rejects bandwidth parts that do not share its numerology.
 *
 * RB width is SCS x 12, so RB counts at different subcarrier spacings are not the
 * same unit and the carrier total would be meaningless; the symbol grids would
 * not line up either. The model carries a single symbol duration, and
 * SetSymbolDuration() simply overwrites, so without this check the last listener
 * to attach would silently win.
 */
class NrGnbEnergyModelMixedNumerologyTestCase : public TestCase
{
  public:
    NrGnbEnergyModelMixedNumerologyTestCase()
        : TestCase("NrGnbEnergyModel rejects BWPs of a carrier with different numerologies")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelMixedNumerologyTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->Initialize();

    NrGnbEnergyModel::BwpOccupancy mu0;
    mu0.rbCount = 100;
    mu0.dlDataMask = 0x3FFF;
    mu0.dlDataReg = 100 * 14;
    mu0.symbolsPerSlot = 14;
    mu0.symbolDuration = NanoSeconds(71429); // numerology 0: 1 ms / 14

    // Same numerology on a second BWP is fine and aggregates normally.
    auto sameMu = mu0;
    gnb->ReportBwpOccupancy(0, mu0);
    gnb->ReportBwpOccupancy(1, sameMu);
    NS_TEST_ASSERT_MSG_GT(gnb->EvaluateCarrierPowerW(),
                          0.0,
                          "matching numerologies must aggregate without complaint");

    // A BWP that reports no numerology at all (the default) must not trip the
    // check, so that hand-built records and older callers keep working.
    NrGnbEnergyModel::BwpOccupancy unspecified;
    unspecified.rbCount = 100;
    unspecified.symbolsPerSlot = 14;
    gnb->ReportBwpOccupancy(2, unspecified);
    NS_TEST_ASSERT_MSG_GT(gnb->EvaluateCarrierPowerW(),
                          0.0,
                          "an unspecified numerology must not be treated as a mismatch");

    // Reporting a DIFFERENT numerology on the same carrier aborts by design: the
    // RB counts could not be summed and the symbol grids would not line up, so
    // any number produced would be meaningless. That path is deliberately fatal
    // and therefore not exercised here - an NS_ABORT cannot be caught in-process.

    Simulator::Destroy();
}

/**
 * @brief FDD: a downlink and an uplink BWP sharing a carrier (TR 38.864 5.1).
 *
 * Reporting symbol POSITIONS rather than counts locates simultaneous DL and UL
 * exactly, so the spec's rule can be applied where it actually applies:
 * "For simultaneous DL and UL transmission for FDD, the power for UL reception
 * is neglected in this study." The uplink-only BWP is also excluded from the DL
 * reference bandwidth, so a saturated downlink still reaches exactly P4.
 */
class NrGnbEnergyModelFddCarrierTestCase : public TestCase
{
  public:
    NrGnbEnergyModelFddCarrierTestCase()
        : TestCase("NrGnbEnergyModel resolves simultaneous DL and UL on one carrier")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelFddCarrierTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->Initialize();

    // An all-downlink BWP alongside an all-uplink one: the FDD layout. Together
    // they claim 28 symbols of a 14-symbol slot.
    NrGnbEnergyModel::BwpOccupancy dl;
    dl.rbCount = 100;
    dl.dlDataMask = 0x3FFF; // all 14 symbols downlink
    dl.dlDataReg = 100 * 14;
    dl.symbolsPerSlot = 14;

    NrGnbEnergyModel::BwpOccupancy ul;
    ul.rbCount = 100;
    ul.ulMask = 0x3FFF; // the same 14 symbols, uplink
    ul.dlCapable = false;
    ul.symbolsPerSlot = 14;

    gnb->ReportBwpOccupancy(0, dl);
    gnb->ReportBwpOccupancy(1, ul);

    // Every symbol carries both directions. TR 38.864 Section 5.1 resolves it:
    // "For simultaneous DL and UL transmission for FDD, the power for UL
    // reception is neglected in this study." So the carrier draws P_DL, and the
    // uplink-only BWP does not widen the sf denominator - the downlink BWP alone
    // is the DL reference bandwidth, so a saturated DL still reaches exactly P4.
    const double p4 = gnb->GetRelativePower(NrGnbPowerState::ActiveDl);
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->EvaluateCarrierPowerW(),
                              p4,
                              1e-9,
                              "simultaneous DL and UL draws P_DL with the UL neglected, and "
                              "a saturated DL must still reach exactly P4");

    // The study assumption is overridable, because a real FDD base station does
    // consume power receiving while transmitting. Only the UL DYNAMIC part is
    // added: P_UL carries the same P3 baseline as P_DL.
    Ptr<NrGnbEnergyModel> both = CreateObject<NrGnbEnergyModel>();
    both->SetAttribute("NeglectUlDuringDl", BooleanValue(false));
    both->Initialize();
    both->ReportBwpOccupancy(0, dl);
    both->ReportBwpOccupancy(1, ul);
    const double p3 = both->GetRelativePower(NrGnbPowerState::MicroSleep);
    const double p5 = both->GetRelativePower(NrGnbPowerState::ActiveUl);
    NS_TEST_ASSERT_MSG_EQ_TOL(both->EvaluateCarrierPowerW(),
                              p4 + (p5 - p3),
                              1e-9,
                              "with the assumption off, only the UL dynamic part is added, "
                              "so P3 is not counted twice");

    // UL alone is still charged at P_UL: it is neglected only when it OVERLAPS DL.
    Ptr<NrGnbEnergyModel> ulOnly = CreateObject<NrGnbEnergyModel>();
    ulOnly->Initialize();
    ulOnly->ReportBwpOccupancy(0, ul);
    NS_TEST_ASSERT_MSG_EQ_TOL(ulOnly->EvaluateCarrierPowerW(),
                              ulOnly->CalcUlPowerW(1.0),
                              1e-9,
                              "uplink on its own still costs P_UL");

    Simulator::Destroy();
}

/**
 * @brief Multi-carrier aggregation (TR 38.864 Section 5.1).
 *
 * "For multi-carrier, the total power consumption of BS is calculated as is the
 * sum of the power consumption of each CC; for intra-band multi-carrier with
 * contiguous CCs, the power consumption of each additional CC is scaled by 0.7."
 *
 * Each carrier keeps its own full power, P3 baseline included: the sharing
 * between carriers is the 0.7 factor, not a dropped baseline. This is the
 * opposite of how BWPs combine inside one carrier, and the two must not be
 * confused.
 */
class NrGnbEnergyAggregatorTestCase : public TestCase
{
  public:
    NrGnbEnergyAggregatorTestCase()
        : TestCase("NrGnbEnergyAggregator sums carrier power with the TR 38.864 weights")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyAggregatorTestCase::DoRun()
{
    // Cat 1 / Set 1 defaults: an idle carrier sits at P3 = 55 W (PowerUnit 1 W).
    const double p3 = 55.0;

    // Two carriers, intra-band contiguous: the anchor at 1.0, the additional at 0.7.
    Ptr<NrGnbEnergyModel> cc0 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyModel> cc1 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
    agg->AddCarrierWithWeight(cc0, 1.0);
    agg->AddCarrierWithWeight(cc1, 0.7);
    agg->Initialize();

    NS_TEST_ASSERT_MSG_EQ(agg->GetNCarriers(), 2, "both carriers registered");
    NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                              p3 + 0.7 * p3, // 93.5
                              1e-9,
                              "two idle intra-band contiguous carriers: P3 + 0.7 x P3");

    // Neither the plain sum nor a single carrier: the weight has to be applied,
    // and the baseline is NOT collapsed to one.
    NS_TEST_ASSERT_MSG_GT(std::abs(agg->ComputeDevicePowerW() - 2 * p3),
                          1.0,
                          "must not be the unweighted sum");
    NS_TEST_ASSERT_MSG_GT(std::abs(agg->ComputeDevicePowerW() - p3),
                          1.0,
                          "must not collapse the baseline to a single carrier");

    // Inter-band: no discount, so both carriers count fully.
    Ptr<NrGnbEnergyModel> b0 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyModel> b1 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyAggregator> inter = CreateObject<NrGnbEnergyAggregator>();
    inter->AddCarrierWithWeight(b0, 1.0);
    inter->AddCarrierWithWeight(b1, 1.0);
    inter->Initialize();
    NS_TEST_ASSERT_MSG_EQ_TOL(inter->ComputeDevicePowerW(),
                              2 * p3,
                              1e-9,
                              "two idle inter-band carriers get no 0.7 discount");

    // Loading one carrier moves the device power by that carrier's weighted share.
    // A fully occupied carrier draws exactly P4 = 280 W.
    NrGnbEnergyModel::BwpOccupancy full;
    full.rbCount = 100;
    full.dlDataMask = 0x3FFF;
    full.dlDataReg = 100 * 14;
    full.symbolsPerSlot = 14;
    cc1->ReportBwpOccupancy(0, full);
    NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                              p3 + 0.7 * 280.0, // 55 + 196 = 251
                              1e-9,
                              "the loaded carrier enters at its own weight");

    Simulator::Destroy();
}

/**
 * @brief The 0.7 is DERIVED from the spectrum, not supplied (TR 38.864 5.1).
 *
 * "For intra-band multi-carrier with contiguous CCs, the power consumption of
 * each additional CC is scaled by 0.7." Being a rule of the specification, it
 * belongs with the formulas: the caller says where each carrier sits and the
 * model decides what that costs. This pins the three ways a carrier can fail to
 * earn the discount - a different band, a gap in the same band, or being the
 * first of a run - and that the answer does not depend on registration order.
 */
class NrGnbEnergyAggregatorContiguityTestCase : public TestCase
{
  public:
    NrGnbEnergyAggregatorContiguityTestCase()
        : TestCase("NrGnbEnergyAggregator derives the contiguity weights from the spectrum")
    {
    }

  private:
    void DoRun() override;

    /// A carrier of width w starting at f, in band b.
    static Ptr<NrGnbEnergyModel> Cc(Ptr<NrGnbEnergyAggregator> agg, uint8_t b, double f, double w)
    {
        Ptr<NrGnbEnergyModel> m = CreateObject<NrGnbEnergyModel>();
        agg->AddCarrier(m, b, f, f + w);
        return m;
    }
};

void
NrGnbEnergyAggregatorContiguityTestCase::DoRun()
{
    const double p3 = 55.0;  // Cat 1 / Set 1 idle power
    const double bw = 100e6; // 100 MHz carriers, laid out edge to edge

    // Three carriers of one band, touching: 3.5 GHz - 3.6 - 3.7 - 3.8.
    // One anchor and two additional CCs.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.5e9, bw);
        Cc(agg, 1, 3.6e9, bw);
        Cc(agg, 1, 3.7e9, bw);
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(0), 1.0, 1e-12, "the anchor keeps 1.0");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1), 0.7, 1e-12, "second CC of the run");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(2), 0.7, 1e-12, "third CC of the run");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                                  p3 + 0.7 * p3 + 0.7 * p3, // 132
                                  1e-9,
                                  "P3 + 0.7 P3 + 0.7 P3, the baseline kept per carrier");
    }

    // Same band, but a 100 MHz gap between them. Not contiguous, so no discount:
    // this is the case a naive "same band => 0.7" test would get wrong.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.5e9, bw);
        Cc(agg, 1, 3.7e9, bw);
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1),
                                  1.0,
                                  1e-12,
                                  "a gap in the same band earns no discount");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(), 2 * p3, 1e-9, "so both count fully");
    }

    // Touching edges but different bands: inter-band CA, no discount.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.5e9, bw);
        Cc(agg, 2, 3.6e9, bw);
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1),
                                  1.0,
                                  1e-12,
                                  "adjacent frequencies in DIFFERENT bands are not intra-band");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(), 2 * p3, 1e-9, "no discount");
    }

    // Registration order must not matter: the anchor is the lowest carrier of the
    // run, whichever order the helper happened to walk the band in.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.7e9, bw); // registered first, but highest
        Cc(agg, 1, 3.5e9, bw); // registered last, but lowest
        Cc(agg, 1, 3.6e9, bw);
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1),
                                  1.0,
                                  1e-12,
                                  "the LOWEST carrier is the anchor, not the first registered");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(0), 0.7, 1e-12, "highest is additional");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(2), 0.7, 1e-12, "middle is additional");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                                  p3 + 0.7 * p3 + 0.7 * p3,
                                  1e-9,
                                  "and the total is order-independent");
    }

    // Two bands at once: a contiguous pair in band 1, a lone carrier in band 2.
    // Each band is scanned separately, so band 2 gets its own anchor.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.5e9, bw);
        Cc(agg, 1, 3.6e9, bw);
        Cc(agg, 2, 28.0e9, bw);
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(2),
                                  1.0,
                                  1e-12,
                                  "a band of its own always starts a new run");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                                  p3 + 0.7 * p3 + p3, // 148.5
                                  1e-9,
                                  "one discounted CC, two anchors");
    }

    // A carrier added later can extend an existing run, which is why the weights
    // are re-derived over the whole set rather than fixed at registration.
    {
        Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
        Cc(agg, 1, 3.5e9, bw);
        Cc(agg, 1, 3.7e9, bw); // a gap for now, so an anchor
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1), 1.0, 1e-12, "anchor while isolated");

        Cc(agg, 1, 3.6e9, bw); // fills the gap: now all three touch
        agg->Initialize();

        NS_TEST_ASSERT_MSG_EQ_TOL(agg->GetCarrierWeight(1),
                                  0.7,
                                  1e-12,
                                  "filling the gap demotes the former anchor to an additional CC");
        NS_TEST_ASSERT_MSG_EQ_TOL(agg->ComputeDevicePowerW(),
                                  p3 + 0.7 * p3 + 0.7 * p3,
                                  1e-9,
                                  "one run of three");
    }

    Simulator::Destroy();
}

/**
 * @brief The aggregator and its EnergySource must account the same energy.
 *
 * The reconciliation established for the single-carrier model has to survive
 * aggregation. The specific way it could break is by appending the per-carrier
 * models to the source as well: BasicEnergySource sums DoGetCurrentA() without
 * weights, so it would integrate the unweighted sum while the aggregator reports
 * the weighted one.
 */
class NrGnbEnergyAggregatorSourceTestCase : public TestCase
{
  public:
    NrGnbEnergyAggregatorSourceTestCase()
        : TestCase("NrGnbEnergyAggregator and its BasicEnergySource agree")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyAggregatorSourceTestCase::DoRun()
{
    const double initialJ = 1e6;

    Ptr<energy::BasicEnergySource> src = CreateObject<energy::BasicEnergySource>();
    src->SetInitialEnergy(initialJ);
    src->SetSupplyVoltage(12.0); // non-unity, so a W/A confusion would show

    Ptr<NrGnbEnergyModel> cc0 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyModel> cc1 = CreateObject<NrGnbEnergyModel>();
    Ptr<NrGnbEnergyAggregator> agg = CreateObject<NrGnbEnergyAggregator>();
    agg->AddCarrierWithWeight(cc0, 1.0);
    agg->AddCarrierWithWeight(cc1, 0.7);

    // ONLY the aggregator is appended. The carriers keep a null source.
    agg->SetEnergySource(src);
    src->AppendDeviceEnergyModel(agg);
    agg->Initialize();

    // Load one carrier part-way through, so the device power really changes.
    NrGnbEnergyModel::BwpOccupancy busy;
    busy.rbCount = 100;
    busy.dlDataMask = 0x3FFF;
    busy.dlDataReg = 100 * 14;
    busy.symbolsPerSlot = 14;
    Simulator::Schedule(MilliSeconds(100), &NrGnbEnergyModel::ReportBwpOccupancy, cc0, 0, busy);

    double modelJ = 0.0;
    double drainedJ = 0.0;
    Simulator::Schedule(MilliSeconds(300), [&]() {
        modelJ = agg->GetTotalEnergyJ();
        drainedJ = initialJ - src->GetRemainingEnergy();
    });
    Simulator::Stop(MilliSeconds(301));
    Simulator::Run();

    // An independent analytical reference. Model-vs-source agreement alone cannot
    // catch a commit-ordering error: if the new power were installed BEFORE the
    // interval was committed, both sides would charge the elapsed time at the new
    // power and still agree with each other. Only a third, independently derived
    // number exposes that.
    //   [0, 100ms)   both carriers idle:  1.0 x 55 + 0.7 x 55        =  93.5 W
    //   [100, 300ms) cc0 saturated:       1.0 x 280 + 0.7 x 55       = 318.5 W
    const double expectedJ = 93.5 * 0.100 + 318.5 * 0.200; // 73.05 J

    NS_TEST_ASSERT_MSG_GT(modelJ, 0.0, "the device must consume energy");
    NS_TEST_ASSERT_MSG_EQ_TOL(modelJ,
                              expectedJ,
                              expectedJ * 1e-9,
                              "the aggregator must charge each interval at the power drawn over "
                              "it, so the commit has to precede the new power being installed");
    NS_TEST_ASSERT_MSG_EQ_TOL(drainedJ,
                              modelJ,
                              modelJ * 1e-9,
                              "the source must drain exactly what the aggregator accounts");

    // And the total is the weighted one, not the unweighted sum of the carriers.
    const double unweighted = cc0->GetTotalEnergyJ() + cc1->GetTotalEnergyJ();
    NS_TEST_ASSERT_MSG_GT(std::abs(modelJ - unweighted),
                          modelJ * 1e-6,
                          "the device total must differ from the unweighted carrier sum");

    Simulator::Destroy();
}

/**
 * @brief TR 38.864 Section 5.1 gNB power formulas.
 */
class NrGnbEnergyModelFormulaTestCase : public TestCase
{
  public:
    NrGnbEnergyModelFormulaTestCase()
        : TestCase("NrGnbEnergyModel TR 38.864 power formulas")
    {
    }

  private:
    void DoRun() override;
};

void
NrGnbEnergyModelFormulaTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();

    // TR 38.864 Table 5.1-3, BS Category 1 / Set 1, PowerUnit = 1 W.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetRelativePower(NrGnbPowerState::DeepSleep), 1.0, 1e-9, "P1");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetRelativePower(NrGnbPowerState::LightSleep), 25.0, 1e-9, "P2");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetRelativePower(NrGnbPowerState::MicroSleep), 55.0, 1e-9, "P3");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetRelativePower(NrGnbPowerState::ActiveDl), 280.0, 1e-9, "P4");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetRelativePower(NrGnbPowerState::ActiveUl), 110.0, 1e-9, "P5");

    // P_DL = P3 + sa*(P4-P3) * [A + (sf*sp/eta)*(1-A)], A = 0.4, eta = 1.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcDlPowerW(1.0, 1.0, 1.0),
                              280.0,
                              1e-9,
                              "full load reaches P4");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcDlPowerW(1.0, 0.0, 1.0),
                              145.0,
                              1e-9,
                              "sf=0 leaves the antenna static part");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcDlPowerW(0.5, 1.0, 1.0),
                              167.5,
                              1e-9,
                              "sa scales the dynamic part");

    // P_UL = P3 + sa*(P5-P3).
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->CalcUlPowerW(1.0), 110.0, 1e-9, "full UL reaches P5");

    // PA efficiency: single mode is constant, dual mode drops below sf*sp = 0.5.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetEta(0.4, 1.0), 1.0, 1e-9, "single eta");
    gnb->SetAttribute("EtaMode", EnumValue(NrGnbEnergyModel::EtaDual));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetEta(0.4, 1.0), 0.76, 1e-9, "dual eta, low load");
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetEta(0.6, 1.0), 1.0, 1e-9, "dual eta, high load");
}

/**
 * @brief Symbol-level slot energy accumulation (TR 38.864 Section 5.2).
 */
class NrGnbEnergyModelSlotAccumTestCase : public TestCase
{
  public:
    NrGnbEnergyModelSlotAccumTestCase()
        : TestCase("NrGnbEnergyModel symbol-level slot accumulation")
    {
    }

  private:
    void DoRun() override;
    /// Assert the committed slot energy once the slot has actually elapsed.
    void CheckCommitted(double expectedJ);

    Ptr<NrGnbEnergyModel> m_gnb; //!< Model under test
};

void
NrGnbEnergyModelSlotAccumTestCase::CheckCommitted(double expectedJ)
{
    NS_TEST_ASSERT_MSG_EQ_TOL(m_gnb->GetTotalEnergyJ(),
                              expectedJ,
                              1e-12,
                              "full DL slot energy committed");
}

void
NrGnbEnergyModelSlotAccumTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel>& gnb = m_gnb;
    gnb = CreateObject<NrGnbEnergyModel>();

    // Numerology 1: slot = 0.5 ms, symbol = 0.5 ms / 14. GetSymbolDuration()
    // returns an ns-3 Time, which is quantized to the simulator resolution
    // (1 ns), so allow a sub-nanosecond tolerance rather than exact real
    // arithmetic. The 14-symbol slot energy below is computed from this actual
    // quantized tSym so the assertion tracks the model's own time base.
    double tSym = gnb->GetSymbolDuration().GetSeconds();
    NS_TEST_ASSERT_MSG_EQ_TOL(tSym, 0.5e-3 / 14.0, 1e-9, "symbol duration for mu=1");
    double slotS = tSym * 14.0; // slot duration actually used by the accumulation

    // Pure helper: a fully idle slot (no allocation) costs 14 symbols at P3.
    std::vector<NrGnbSymbolType> idleSlot(14, NrGnbSymbolType::Idle);
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->ComputeSlotEnergyJ(idleSlot, 1.0, 0.0, 1.0),
                              55.0 * slotS,
                              1e-12,
                              "all-idle slot at P3");

    // Accumulation path: a full-load DL slot commits 14 symbols at P4.
    for (uint32_t s = 0; s < 14; ++s)
    {
        gnb->UpdateSymbolPower(1.0, 1.0, 1.0, NrGnbSymbolType::Dl);
    }
    gnb->FinalizeSlotEnergy();
    // FinalizeSlotEnergy() converts the accumulated symbols into an average power
    // for the slot that follows, rather than adding the energy up front, so the
    // total is only observable once that slot has actually elapsed.
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetTotalEnergyJ(),
                              0.0,
                              1e-12,
                              "no energy is committed in advance of the slot");
    Simulator::Schedule(Seconds(slotS),
                        &NrGnbEnergyModelSlotAccumTestCase::CheckCommitted,
                        this,
                        280.0 * slotS);
    Simulator::Stop(Seconds(slotS));
    Simulator::Run();
    Simulator::Destroy();
    m_gnb = nullptr;
}

/**
 * @brief DRX onDuration / inactivity / sleep-depth cycling.
 */
class NrUeDrxModelCycleTestCase : public TestCase
{
  public:
    NrUeDrxModelCycleTestCase()
        : TestCase("NrUeDrxModel DRX cycle drives the UE power states")
    {
    }

  private:
    void DoRun() override;

    /**
     * @brief Assert the UE power state at the current simulation time.
     * @param expected Expected state.
     * @param context  Failure message context.
     */
    void CheckState(NrUePowerState expected, std::string context);

    Ptr<NrUeEnergyModel> m_ue; //!< Driven energy model
    Ptr<NrUeDrxModel> m_drx;   //!< DRX model under test
};

void
NrUeDrxModelCycleTestCase::CheckState(NrUePowerState expected, std::string context)
{
    NS_TEST_ASSERT_MSG_EQ(m_ue->GetCurrentState(), expected, context);
}

void
NrUeDrxModelCycleTestCase::DoRun()
{
    m_ue = CreateObject<NrUeEnergyModel>();
    m_drx = CreateObject<NrUeDrxModel>();
    m_drx->SetEnergyModel(m_ue);

    // Defaults: long cycle 160 ms, onDuration 8 ms, inactivity 10 ms,
    // deep-sleep threshold 20 ms.
    m_drx->Start(Seconds(0));

    // Inside the first onDuration: PDCCH monitoring.
    Simulator::Schedule(MilliSeconds(4),
                        &NrUeDrxModelCycleTestCase::CheckState,
                        this,
                        NR_UE_PDCCH_ONLY,
                        std::string("onDuration monitors PDCCH"));
    // After onDuration with no activity: gap 152 ms >= 20 ms -> deep sleep.
    Simulator::Schedule(MilliSeconds(12),
                        &NrUeDrxModelCycleTestCase::CheckState,
                        this,
                        NR_UE_DEEP_SLEEP,
                        std::string("sleeps deep after an idle onDuration"));
    // Second cycle wakes at 160 ms.
    Simulator::Schedule(MilliSeconds(164),
                        &NrUeDrxModelCycleTestCase::CheckState,
                        this,
                        NR_UE_PDCCH_ONLY,
                        std::string("second cycle wakes the UE"));
    // Data activity at 166 ms arms the inactivity timer (expires at 176 ms).
    Simulator::Schedule(MilliSeconds(166), &NrUeDrxModel::NotifyDataActivity, m_drx);
    // Past the onDuration end (168 ms) the inactivity timer keeps the UE awake.
    Simulator::Schedule(MilliSeconds(172),
                        &NrUeDrxModelCycleTestCase::CheckState,
                        this,
                        NR_UE_PDCCH_ONLY,
                        std::string("inactivity timer keeps the UE awake"));
    // After the inactivity expiry: gap to 320 ms is 144 ms -> deep sleep.
    Simulator::Schedule(MilliSeconds(180),
                        &NrUeDrxModelCycleTestCase::CheckState,
                        this,
                        NR_UE_DEEP_SLEEP,
                        std::string("sleeps again after inactivity expiry"));

    Simulator::Stop(MilliSeconds(200));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @brief Test suite for the NR energy models and the DRX power model.
 */
class NrEnergyModelsTestSuite : public TestSuite
{
  public:
    NrEnergyModelsTestSuite()
        : TestSuite("nr-energy-models", Type::UNIT)
    {
        AddTestCase(new NrUeEnergyModelPowerTestCase(), Duration::QUICK);
        AddTestCase(new NrUeEnergyModelRxChainScalingTestCase(), Duration::QUICK);
        AddTestCase(new NrUeEnergyModelTransientTestCase(), Duration::QUICK);
        AddTestCase(new NrUeEnergyModelTable19TestCase(), Duration::QUICK);
        AddTestCase(new NrUeEnergyModelAccountingTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelPowerTableTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelRefConfigBundleTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelTransitionTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelSfNormalizationTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelCarrierAggregationTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelMixedNumerologyTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelFddCarrierTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyAggregatorTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyAggregatorContiguityTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyAggregatorSourceTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelFormulaTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelSlotAccumTestCase(), Duration::QUICK);
        AddTestCase(new NrUeDrxModelCycleTestCase(), Duration::QUICK);
    }
};

static NrEnergyModelsTestSuite g_nrEnergyModelsTestSuite; //!< the test suite

} // namespace ns3
