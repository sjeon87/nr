#pragma once

#include <ns3/flow-monitor-helper.h>
#include <ns3/flow-monitor.h>
#include <ns3/ptr.h>

#include <string>

namespace ns3
{

/**
 * \brief Aggregated per-flow stats returned by NrFlowMonitorPrintStats.
 */
struct NrFlowMonitorStats
{
    double meanFlowThroughputMbps{0.0}; //!< Mean throughput across all flows (Mbps)
    double meanFlowDelayMs{0.0};        //!< Mean delay across all flows (ms)
};

/**
 * \brief Print per-flow statistics from a FlowMonitor to a file.
 *
 * This is a function that replaces the FlowMonitor stats
 * printing code duplicated across many NR examples.  It iterates
 * over every flow collected by \p monitor, writes per-flow Tx/Rx/throughput/
 * delay/jitter information to \p outputFilePath, and optionally echoes the
 * same output to stdout.
 *
 * \param monitor        The FlowMonitor instance (after Simulator::Run).
 * \param flowmonHelper  The FlowMonitorHelper used to install the monitor.
 * \param flowDurationSeconds  Duration over which to compute throughput (seconds).
 * \param outputFilePath Full path of the output file (e.g. "results/sim1").
 * \param printToStdout  If true, print the file contents to std::cout after writing.
 * \return Aggregated statistics (mean throughput and mean delay).
 */
NrFlowMonitorStats NrFlowMonitorPrintStats(const Ptr<FlowMonitor>& monitor,
                                           FlowMonitorHelper& flowmonHelper,
                                           double flowDurationSeconds,
                                           const std::string& outputFilePath,
                                           bool printToStdout = true);

} // namespace ns3
