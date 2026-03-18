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

struct NrFlowSummary
{
  uint32_t flowCount{0};
  double throughputMbps{0.0};
  double meanDelayMs{0.0};
  double lossRatio{0.0};
};

class NrMetricsHelper : public Object
{
public:
  static TypeId GetTypeId();

  void EnableStandardTraces(Ptr<NrHelper> nrHelper, bool enable);

  Ptr<FlowMonitor> InstallFlowMonitor(const NodeContainer& endpoints);

  NrFlowSummary ComputeFlowSummary(Ptr<FlowMonitor> monitor,
                                   Ptr<Ipv4FlowClassifier> classifier) const;

private:
  FlowMonitorHelper m_flowmonHelper;
};

} // namespace ns3

#endif // NR_METRICS_HELPER_H