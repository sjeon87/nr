// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)

#include "ns3/enum.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-ue-drx-model.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

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
    NS_TEST_ASSERT_MSG_EQ_TOL(fr2->GetRelativePower(NR_UE_MICRO_SLEEP), 38.0, 1e-9, "FR2 micro");
    fr2->SetUlTxPowerDbm(5.0);
    NS_TEST_ASSERT_MSG_EQ_TOL(fr2->GetRelativePower(NR_UE_UL_TX),
                              350.0,
                              1e-9,
                              "FR2 UL is level-independent");
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

    // E = 1e-3*1 + 0.1*1 + 1e-3*1 = 0.102 J (open interval included).
    NS_TEST_ASSERT_MSG_EQ_TOL(m_ue->GetTotalEnergyJ(), 0.102, 1e-9, "integrated energy");
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
};

void
NrGnbEnergyModelSlotAccumTestCase::DoRun()
{
    Ptr<NrGnbEnergyModel> gnb = CreateObject<NrGnbEnergyModel>();

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
    NS_TEST_ASSERT_MSG_EQ_TOL(gnb->GetTotalEnergyJ(),
                              280.0 * slotS,
                              1e-12,
                              "full DL slot energy committed");

    Simulator::Destroy();
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
        AddTestCase(new NrUeEnergyModelAccountingTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelFormulaTestCase(), Duration::QUICK);
        AddTestCase(new NrGnbEnergyModelSlotAccumTestCase(), Duration::QUICK);
        AddTestCase(new NrUeDrxModelCycleTestCase(), Duration::QUICK);
    }
};

static NrEnergyModelsTestSuite g_nrEnergyModelsTestSuite; //!< the test suite

} // namespace ns3
