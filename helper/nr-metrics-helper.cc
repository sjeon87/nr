// SPDX-License-Identifier: GPL-2.0-only

#include "nr-metrics-helper.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMetricsHelper");
NS_OBJECT_ENSURE_REGISTERED(NrMetricsHelper);

TypeId
NrMetricsHelper::GetTypeId()
{
  static TypeId tid = TypeId("ns3::NrMetricsHelper")
                        .SetParent<Object>()
                        .SetGroupName("Nr")
                        .AddConstructor<NrMetricsHelper>();
  return tid;
}

void
NrMetricsHelper::EnableStandardTraces(Ptr<NrHelper> nrHelper, bool enable)
{
  NS_ABORT_MSG_IF(nrHelper == nullptr, "NrMetricsHelper: nrHelper is null");
  // Keep trace activation logic centralized for consistency across examples.
  if (enable)
  {
    nrHelper->EnableTraces();
  }
}

Ptr<FlowMonitor>
NrMetricsHelper::InstallFlowMonitor(const NodeContainer& endpoints)
{
  return m_flowmonHelper.Install(endpoints);
}

NrFlowSummary
NrMetricsHelper::ComputeFlowSummary(Ptr<FlowMonitor> monitor,
                                    Ptr<Ipv4FlowClassifier> classifier) const
{
  NS_ABORT_MSG_IF(monitor == nullptr, "NrMetricsHelper: monitor is null");

  NrFlowSummary out;
  // Standard post-run flow-monitor checks shared by many NR examples.
  monitor->CheckForLostPackets();
  const auto stats = monitor->GetFlowStats();
  out.flowCount = stats.size();

  double sumThroughputMbps = 0.0;
  double sumDelayMs = 0.0;
  double totalRxPackets = 0.0;
  double totalLostPackets = 0.0;

  for (const auto& kv : stats)
  {
    const auto& st = kv.second;
    const double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
    if (duration > 0.0)
    {
      sumThroughputMbps += (st.rxBytes * 8.0) / duration / 1e6;
    }
    if (st.rxPackets > 0)
    {
      sumDelayMs += 1000.0 * st.delaySum.GetSeconds() / st.rxPackets;
    }
    totalRxPackets += st.rxPackets;
    totalLostPackets += st.lostPackets;

    if (classifier)
    {
      (void)classifier->FindFlow(kv.first);
    }
  }

  if (!stats.empty())
  {
    out.throughputMbps = sumThroughputMbps;
    out.meanDelayMs = sumDelayMs / stats.size();
  }
  if (totalRxPackets + totalLostPackets > 0.0)
  {
    out.lossRatio = totalLostPackets / (totalRxPackets + totalLostPackets);
  }
  return out;
}

} // namespace ns3