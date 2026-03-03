
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
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    outFile.open(outputFilePath.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << outputFilePath << std::endl;
        return {};
    }

    outFile.setf(std::ios_base::fixed);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  "
                << i->second.txBytes * 8.0 / flowDurationSeconds / 1000.0 / 1000.0 << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput +=
                i->second.rxBytes * 8.0 / flowDurationSeconds / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: "
                    << i->second.rxBytes * 8.0 / flowDurationSeconds / 1000 / 1000 << " Mbps\n";
            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    NrFlowMonitorStats result;
    if (!stats.empty())
    {
        result.meanFlowThroughputMbps = averageFlowThroughput / stats.size();
        result.meanFlowDelayMs = averageFlowDelay / stats.size();
    }

    outFile << "\n\n  Mean flow throughput: " << result.meanFlowThroughputMbps << "\n";
    outFile << "  Mean flow delay: " << result.meanFlowDelayMs << "\n";

    outFile.close();

    if (printToStdout)
    {
        std::ifstream f(outputFilePath.c_str());
        if (f.is_open())
        {
            std::cout << f.rdbuf();
        }
    }

    return result;
}

} // namespace ns3
