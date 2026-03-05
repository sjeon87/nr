
#include "nr-flow-monitor-output-helper.h"

#include <ns3/flow-monitor.h>
#include <ns3/ipv4-flow-classifier.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace ns3
{

NrFlowMonitorStats
NrFlowMonitorPrintStats(const Ptr<FlowMonitor>& monitor,
                        FlowMonitorHelper& flowmonHelper,
                        double flowDurationSeconds,
                        const std::string& outputFilePath,
                        bool printToStdout)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    Time flowDuration = Seconds(flowDurationSeconds);
    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile(outputFilePath, std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << outputFilePath << std::endl;
        return {};
    }

    outFile.setf(std::ios_base::fixed);

    for (const auto& [flowId, stats] : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        std::stringstream protoStream;
        protoStream << static_cast<uint16_t>(t.protocol);
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << flowId << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << stats.txPackets << "\n";
        outFile << "  Tx Bytes:   " << stats.txBytes << "\n";
        outFile << "  TxOffered:  " << stats.GetTxOfferedLoad(flowDuration) / 1e6 << " Mbps\n";
        outFile << "  Rx Bytes:   " << stats.rxBytes << "\n";
        if (stats.rxPackets > 0)
        {
            double throughputMbps = stats.GetRxThroughput(flowDuration) / 1e6;
            double delayMs = stats.GetMeanDelay().GetMilliSeconds();
            double jitterMs = stats.GetMeanJitter().GetMilliSeconds();

            averageFlowThroughput += throughputMbps;
            averageFlowDelay += delayMs;

            outFile << "  Throughput: " << throughputMbps << " Mbps\n";
            outFile << "  Mean delay:  " << delayMs << " ms\n";
            outFile << "  Mean jitter:  " << jitterMs << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << stats.rxPackets << "\n";
    }

    NrFlowMonitorStats result;
    uint32_t flowCount = monitor->GetFlowStats().size();
    if (flowCount > 0)
    {
        result.meanFlowThroughputMbps = averageFlowThroughput / flowCount;
        result.meanFlowDelayMs = averageFlowDelay / flowCount;
    }

    outFile << "\n\n  Mean flow throughput: " << result.meanFlowThroughputMbps << "\n";
    outFile << "  Mean flow delay: " << result.meanFlowDelayMs << "\n";

    outFile.close();

    if (printToStdout)
    {
        std::ifstream f(outputFilePath);
        if (f.is_open())
        {
            std::cout << f.rdbuf();
        }
    }

    return result;
}

} // namespace ns3
