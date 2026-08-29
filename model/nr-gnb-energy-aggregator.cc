// Copyright (c) 2026 University of Moratuwa
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nipuna Dulara (nipuna.21@cse.mrt.ac.lk)
//
// 3GPP References:
//   TR 38.864 V18.1.0 (2023-03): Section 5.1 - multi-carrier BS power consumption

#include "nr-gnb-energy-aggregator.h"

#include "ns3/double.h"
#include "ns3/energy-source.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbEnergyAggregator");
NS_OBJECT_ENSURE_REGISTERED(NrGnbEnergyAggregator);

TypeId
NrGnbEnergyAggregator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrGnbEnergyAggregator")
            .SetParent<energy::DeviceEnergyModel>()
            .SetGroupName("Nr")
            .AddConstructor<NrGnbEnergyAggregator>()
            .AddAttribute("ContiguityToleranceHz",
                          "How closely two carrier edges must meet to count as intra-band "
                          "contiguous, and so attract the TR 38.864 0.7 scaling. The default "
                          "admits only carriers laid out edge to edge, as CcBwpCreator "
                          "produces them; widen it for a topology with a guard band between "
                          "contiguous CCs.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrGnbEnergyAggregator::m_contiguityToleranceHz),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("InstantaneousPower",
                            "Weighted device power draw [W]. Fires whenever any carrier's "
                            "power changes.",
                            MakeTraceSourceAccessor(&NrGnbEnergyAggregator::m_powerTrace),
                            "ns3::TracedValueCallback::Double")
            .AddTraceSource("TotalEnergyConsumption",
                            "Total energy consumed by the device [J].",
                            MakeTraceSourceAccessor(&NrGnbEnergyAggregator::m_totalEnergyJ),
                            "ns3::TracedValueCallback::Double");
    return tid;
}

NrGnbEnergyAggregator::NrGnbEnergyAggregator()
    : m_source(nullptr),
      m_contiguityToleranceHz(1.0),
      m_currentPowerW(0.0),
      m_lastUpdateTime(Seconds(0)),
      m_powerTrace(0.0),
      m_totalEnergyJ(0.0)
{
    NS_LOG_FUNCTION(this);
}

NrGnbEnergyAggregator::~NrGnbEnergyAggregator()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbEnergyAggregator::AddCarrier(Ptr<NrGnbEnergyModel> model,
                                  uint8_t bandId,
                                  double lowerFrequencyHz,
                                  double higherFrequencyHz)
{
    NS_LOG_FUNCTION(this << model << +bandId << lowerFrequencyHz << higherFrequencyHz);
    NS_ASSERT_MSG(model, "A carrier needs an energy model");
    NS_ASSERT_MSG(higherFrequencyHz > lowerFrequencyHz,
                  "Carrier upper edge " << higherFrequencyHz << " Hz must exceed the lower edge "
                                        << lowerFrequencyHz << " Hz");

    RegisterCarrier({model, 1.0, false, bandId, lowerFrequencyHz, higherFrequencyHz});
}

void
NrGnbEnergyAggregator::AddCarrierWithWeight(Ptr<NrGnbEnergyModel> model, double weight)
{
    NS_LOG_FUNCTION(this << model << weight);
    NS_ASSERT_MSG(model, "A carrier needs an energy model");
    NS_ASSERT_MSG(weight > 0.0, "A carrier weight must be positive, got " << weight);

    RegisterCarrier({model, weight, true, 0, 0.0, 0.0});
}

void
NrGnbEnergyAggregator::RegisterCarrier(const Carrier& carrier)
{
    NS_LOG_FUNCTION(this);
    // Close the interval before the device power changes, exactly as for any
    // other boundary: adding a carrier raises the device power from this instant
    // on, and the time already elapsed was drawn without it. Recomputing the
    // weights can also change what the carriers already present contribute.
    CommitInterval();

    m_carriers.push_back(carrier);

    // The device power is re-read from every carrier on any change, so the
    // callback does not need to know which carrier fired.
    carrier.model->TraceConnectWithoutContext(
        "InstantaneousPower",
        MakeCallback(&NrGnbEnergyAggregator::CarrierPowerChanged, this));

    RecomputeWeights();

    m_currentPowerW = ComputeDevicePowerW();
    m_powerTrace = m_currentPowerW;
}

void
NrGnbEnergyAggregator::RecomputeWeights()
{
    NS_LOG_FUNCTION(this);

    // Walk each band's carriers in frequency order. A run of carriers whose
    // edges meet is one intra-band contiguous group: the first keeps 1.0 and
    // every additional one is scaled by 0.7 (TR 38.864 Section 5.1). A gap, or a
    // different band, starts a new run whose first carrier is again an anchor.
    //
    // Sorting by index rather than reordering m_carriers keeps registration
    // order intact, which is what GetCarrier(i) and GetCarrierWeight(i) report.
    std::map<uint8_t, std::vector<uint32_t>> byBand;
    for (uint32_t i = 0; i < m_carriers.size(); ++i)
    {
        if (!m_carriers[i].explicitWeight)
        {
            byBand[m_carriers[i].bandId].push_back(i);
        }
    }

    for (auto& [bandId, indices] : byBand)
    {
        std::sort(indices.begin(), indices.end(), [this](uint32_t a, uint32_t b) {
            return m_carriers[a].lowerFrequencyHz < m_carriers[b].lowerFrequencyHz;
        });

        double runEdgeHz = 0.0;
        bool inRun = false;

        for (const uint32_t idx : indices)
        {
            Carrier& c = m_carriers[idx];
            const bool touchesRun =
                inRun && (std::abs(c.lowerFrequencyHz - runEdgeHz) <= m_contiguityToleranceHz);

            c.weight = touchesRun ? CONTIGUOUS_CC_SCALING : 1.0;
            NS_LOG_LOGIC("carrier " << idx << " in band " << +bandId << " [" << c.lowerFrequencyHz
                                    << ", " << c.higherFrequencyHz << "] weight " << c.weight);

            // Overlapping carriers would otherwise pull the running edge
            // backwards and break the contiguity test for the next one.
            runEdgeHz = std::max(inRun ? runEdgeHz : c.higherFrequencyHz, c.higherFrequencyHz);
            inRun = true;
        }
    }
}

uint32_t
NrGnbEnergyAggregator::GetNCarriers() const
{
    return static_cast<uint32_t>(m_carriers.size());
}

Ptr<NrGnbEnergyModel>
NrGnbEnergyAggregator::GetCarrier(uint32_t index) const
{
    NS_ASSERT_MSG(index < m_carriers.size(), "Carrier index " << index << " out of range");
    return m_carriers[index].model;
}

double
NrGnbEnergyAggregator::GetCarrierWeight(uint32_t index) const
{
    NS_ASSERT_MSG(index < m_carriers.size(), "Carrier index " << index << " out of range");
    return m_carriers[index].weight;
}

double
NrGnbEnergyAggregator::GetInstantaneousPowerW() const
{
    return ComputeDevicePowerW();
}

double
NrGnbEnergyAggregator::ComputeDevicePowerW() const
{
    // TR 38.864 Section 5.1: the total BS power is the sum of the power of each
    // CC, with each additional intra-band contiguous CC scaled by 0.7. Every
    // carrier contributes its complete power, its own P3 baseline included; the
    // sharing between carriers is expressed by the weight, not by dropping the
    // baseline.
    double sum = 0.0;
    for (const auto& c : m_carriers)
    {
        sum += c.weight * c.model->GetInstantaneousPowerW();
    }
    return sum;
}

void
NrGnbEnergyAggregator::CarrierPowerChanged(double, double)
{
    NS_LOG_FUNCTION(this);
    // Close the elapsed interval at the OLD device power, which is still held in
    // m_currentPowerW, before reading the carriers back. When several carriers
    // change at the same instant, every commit after the first spans zero time,
    // so the intermediate partial sums cost nothing and the final device power is
    // the same whatever order the carriers fire in.
    CommitInterval();
    m_currentPowerW = ComputeDevicePowerW();
    m_powerTrace = m_currentPowerW;
}

void
NrGnbEnergyAggregator::CommitInterval()
{
    NS_LOG_FUNCTION(this);
    const Time now = Simulator::Now();
    m_totalEnergyJ = m_totalEnergyJ + m_currentPowerW * (now - m_lastUpdateTime).GetSeconds();
    m_lastUpdateTime = now;
    if (m_source)
    {
        m_source->UpdateEnergySource();
    }
}

double
NrGnbEnergyAggregator::GetTotalEnergyJ() const
{
    NS_LOG_FUNCTION(this);
    // CommitInterval() is the only writer of m_totalEnergyJ and always advances
    // m_lastUpdateTime, so the committed part and the projection below abut and
    // can neither overlap nor leave a gap.
    const double durationS = (Simulator::Now() - m_lastUpdateTime).GetSeconds();
    return m_totalEnergyJ + m_currentPowerW * durationS;
}

void
NrGnbEnergyAggregator::SetEnergySource(Ptr<energy::EnergySource> source)
{
    NS_LOG_FUNCTION(this << source);
    NS_ASSERT_MSG(source, "The aggregator needs an energy source");
    m_source = source;
}

double
NrGnbEnergyAggregator::GetTotalEnergyConsumption() const
{
    return GetTotalEnergyJ();
}

void
NrGnbEnergyAggregator::ChangeState(int newState)
{
    NS_LOG_FUNCTION(this << newState);
    // A device-level state applies to the whole gNB. Each carrier commits its own
    // interval and installs its own power, and each of those fires the trace that
    // brings CarrierPowerChanged() here to re-aggregate, so no explicit commit is
    // needed around this loop.
    for (auto& c : m_carriers)
    {
        c.model->ChangeState(newState);
    }
}

void
NrGnbEnergyAggregator::HandleEnergyDepletion()
{
    NS_LOG_FUNCTION(this);
    for (auto& c : m_carriers)
    {
        c.model->HandleEnergyDepletion();
    }
}

void
NrGnbEnergyAggregator::HandleEnergyRecharged()
{
    NS_LOG_FUNCTION(this);
    for (auto& c : m_carriers)
    {
        c.model->HandleEnergyRecharged();
    }
}

void
NrGnbEnergyAggregator::HandleEnergyChanged()
{
    NS_LOG_FUNCTION(this);
    for (auto& c : m_carriers)
    {
        c.model->HandleEnergyChanged();
    }
}

double
NrGnbEnergyAggregator::DoGetCurrentA() const
{
    NS_LOG_FUNCTION(this);
    const double voltage = m_source ? m_source->GetSupplyVoltage() : 1.0;
    // The same quantity GetTotalEnergyJ() integrates, so the attached source and
    // this model can never diverge.
    return (voltage > 0.0) ? m_currentPowerW / voltage : 0.0;
}

void
NrGnbEnergyAggregator::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    // Carriers registered before the run starts have not fired their trace yet,
    // so seed the device power from them: the source must drain from t = 0, and
    // TR 38.864 Table 5.1-3 has no zero-power state.
    for (auto& c : m_carriers)
    {
        c.model->Initialize();
    }
    m_currentPowerW = ComputeDevicePowerW();
    m_powerTrace = m_currentPowerW;
    energy::DeviceEnergyModel::DoInitialize();
}

void
NrGnbEnergyAggregator::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_carriers.clear();
    m_source = nullptr;
    energy::DeviceEnergyModel::DoDispose();
}

} // namespace ns3
