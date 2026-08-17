// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_METRICS_HELPER_H
#define NR_METRICS_HELPER_H

#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nr-helper.h"

namespace ns3
{

/**
 * @brief Aggregated flow-monitor KPIs.
 */
struct NrFlowSummary
{
  uint32_t flowCount{0};     //!< Number of monitored flows
  double throughputMbps{0.0}; //!< Sum throughput across flows (Mbit/s)
  double meanDelayMs{0.0};   //!< Mean delay across monitored flows (ms)
  double lossRatio{0.0};     //!< Packet loss ratio
};

/**
 * @ingroup helper
 * @brief Helper that centralizes repetitive trace/flow-monitor setup and KPI extraction.
 *
 * Rationale:
 * many NR examples repeat the same metrics pattern:
 * - EnableTraces()
 * - FlowMonitorHelper installation
 * - post-run KPI extraction and aggregation
 *
 * This helper keeps the metrics workflow consistent across examples and reduces
 * duplicated reporting code.
 */
class NrMetricsHelper : public Object
{
public:
  /**
   * @brief Get TypeId.
   * @return TypeId of NrMetricsHelper.
   */
  static TypeId GetTypeId();

  /**
   * @brief Enable standard NR traces through NrHelper.
   * @param nrHelper NR helper instance.
   * @param enable Whether traces should be enabled.
   */
  void EnableStandardTraces(Ptr<NrHelper> nrHelper, bool enable);

  /**
   * @brief Install flow monitor on selected endpoints.
   * @param endpoints Node container where flow monitor probes are installed.
   * @return FlowMonitor instance.
   */
  Ptr<FlowMonitor> InstallFlowMonitor(const NodeContainer& endpoints);

  /**
   * @brief Compute compact KPI summary from flow monitor statistics.
   * @param monitor Flow monitor object.
   * @param classifier Optional classifier used for flow lookup.
   * @return Aggregated flow summary.
   */
  NrFlowSummary ComputeFlowSummary(Ptr<FlowMonitor> monitor,
                                   Ptr<Ipv4FlowClassifier> classifier) const;

private:
  FlowMonitorHelper m_flowmonHelper;
};

} // namespace ns3

#endif // NR_METRICS_HELPER_H