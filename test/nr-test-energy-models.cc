// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-ue-drx-model.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

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

    // Entering deep sleep from active charges the transition once; waking does not
    // charge it again, since the tabulated value covers both ramp directions.
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();
    gnb->ChangeState(static_cast<int>(NrGnbPowerState::ActiveDl));
    double before = gnb->GetTotalEnergyJ();
    gnb->ChangeState(static_cast<int>(NrGnbPowerState::DeepSleep));
    double afterSleep = gnb->GetTotalEnergyJ();
    NS_TEST_ASSERT_MSG_EQ_TOL(afterSleep - before,
                              1.0,
                              1e-12,
                              "entering deep sleep charges the Table 5.1-5 energy once");
    gnb->ChangeState(static_cast<int>(NrGnbPowerState::ActiveDl));
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetTotalEnergyJ() - afterSleep,
                              0.0,
                              1e-12,
                              "waking must not charge the transition energy a second time");
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
        AddTestCase(new NrUeEnergyModelAccountingTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelPowerTableTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelRefConfigBundleTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelTransitionTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelFormulaTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelSlotAccumTestCase(), Duration::QUICK);
        AddTestCase(new NrUeDrxModelCycleTestCase(), Duration::QUICK);
    }
};

static NrEnergyModelsTestSuite g_nrEnergyModelsTestSuite; //!< the test suite

} // namespace ns3
