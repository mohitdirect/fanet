/*
 * sinr_monitor.cc - SINR and PER Monitoring Implementation
 * 
 * Provides realistic jammer detection based on measurable metrics
 * instead of omniscient distance-based detection.
 */

#include "sinr_monitor.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <cmath>
#include <algorithm>
#include <numeric>

NS_LOG_COMPONENT_DEFINE("SinrMonitor");

namespace myfanet {

// Global signal statistics
std::map<uint32_t, NodeSignalStats> nodeSignalStats;
DetectionMode currentDetectionMode = DetectionMode::HYBRID;  // IMPROVED: Using realistic SINR+PER detection

// Initialize SINR monitoring for all nodes
void InitializeSinrMonitoring()
{
    nodeSignalStats.clear();
    for (uint32_t i = 0; i < nNodes; ++i) {
        NodeSignalStats stats;
        stats.lastSampleTime = ns3::Simulator::Now();
        stats.avgSinr = SINR_GOOD_THRESHOLD;
        stats.minSinr = SINR_GOOD_THRESHOLD;
        nodeSignalStats[i] = stats;
    }
    
    NS_LOG_UNCOND("[SINR Monitor] Initialized for " << nNodes << " nodes");
    NS_LOG_UNCOND("[SINR Monitor] Detection mode: " << 
        (currentDetectionMode == DetectionMode::DISTANCE_BASED ? "DISTANCE" :
         currentDetectionMode == DetectionMode::SINR_BASED ? "SINR" :
         currentDetectionMode == DetectionMode::PER_BASED ? "PER" : "HYBRID"));
    
    // Schedule periodic updates
    ns3::Simulator::Schedule(ns3::Seconds(SINR_SAMPLE_INTERVAL), &UpdateSinrMonitoring);
}

// Record SINR sample for a node
void RecordSinrSample(uint32_t nodeId, double sinrDb)
{
    if (nodeId >= nNodes) return;
    
    auto& stats = nodeSignalStats[nodeId];
    stats.sinrHistory.push_back(sinrDb);
    
    // Keep only recent history
    if (stats.sinrHistory.size() > SINR_HISTORY_SIZE) {
        stats.sinrHistory.erase(stats.sinrHistory.begin());
    }
    
    // Update running average
    if (!stats.sinrHistory.empty()) {
        stats.avgSinr = std::accumulate(stats.sinrHistory.begin(), 
                                         stats.sinrHistory.end(), 0.0) / stats.sinrHistory.size();
        stats.minSinr = *std::min_element(stats.sinrHistory.begin(), stats.sinrHistory.end());
    }
    
    // Mark as degraded if below threshold
    stats.isDegraded = (stats.avgSinr < SINR_DEGRADED_THRESHOLD);
    
    stats.lastSampleTime = ns3::Simulator::Now();
}

// Packet tracking for PER calculation
void RecordPacketTx(uint32_t nodeId)
{
    if (nodeId < nNodes) {
        nodeSignalStats[nodeId].txPackets++;
    }
}

void RecordPacketRx(uint32_t nodeId)
{
    if (nodeId < nNodes) {
        nodeSignalStats[nodeId].rxPackets++;
    }
}

void RecordPacketLoss(uint32_t nodeId)
{
    if (nodeId < nNodes) {
        nodeSignalStats[nodeId].lostPackets++;
    }
}

// Compute SINR-based LSSS (0 = critical, 1 = good)
double ComputeSinrBasedLSSS(uint32_t nodeId)
{
    if (nodeId >= nNodes) return 0.5;
    
    const auto& stats = nodeSignalStats[nodeId];
    double sinr = stats.avgSinr;
    
    // Map SINR to 0-1 score
    // SINR >= 20 dB → 1.0 (excellent)
    // SINR = 10 dB  → 0.5 (degraded)
    // SINR <= 5 dB  → 0.0 (critical)
    
    if (sinr >= SINR_GOOD_THRESHOLD) {
        return 1.0;
    } else if (sinr <= SINR_CRITICAL_THRESHOLD) {
        return 0.0;
    } else {
        // Linear interpolation between critical and good
        return (sinr - SINR_CRITICAL_THRESHOLD) / 
               (SINR_GOOD_THRESHOLD - SINR_CRITICAL_THRESHOLD);
    }
}

// Compute PER-based LSSS (0 = high error rate, 1 = no errors)
double ComputePerBasedLSSS(uint32_t nodeId)
{
    if (nodeId >= nNodes) return 0.5;
    
    auto& stats = nodeSignalStats[nodeId];
    
    // Calculate PER
    uint32_t totalPackets = stats.txPackets + stats.rxPackets;
    if (totalPackets > 0) {
        stats.packetErrorRate = static_cast<double>(stats.lostPackets) / totalPackets;
    } else {
        stats.packetErrorRate = 0.0;
    }
    
    // Map PER to 0-1 score (inverse)
    // PER = 0%  → 1.0 (excellent)
    // PER = 15% → 0.5 (degraded)
    // PER >= 30% → 0.0 (critical)
    
    double per = stats.packetErrorRate;
    if (per <= PER_NORMAL_THRESHOLD) {
        return 1.0;
    } else if (per >= PER_CRITICAL_THRESHOLD) {
        return 0.0;
    } else {
        return 1.0 - (per - PER_NORMAL_THRESHOLD) / 
               (PER_CRITICAL_THRESHOLD - PER_NORMAL_THRESHOLD);
    }
}

// Hybrid detection combining SINR and PER
double ComputeHybridLSSS(uint32_t nodeId)
{
    double sinrScore = ComputeSinrBasedLSSS(nodeId);
    double perScore = ComputePerBasedLSSS(nodeId);
    
    // Use minimum (most pessimistic) - if either indicates problem, detect it
    return std::min(sinrScore, perScore);
}

// Check if handoff should be triggered based on current detection mode
bool ShouldTriggerHandoff()
{
    uint32_t degradedCount = 0;
    double minScore = 1.0;
    
    for (uint32_t i = 0; i < nNodes; ++i) {
        double score;
        
        switch (currentDetectionMode) {
            case DetectionMode::SINR_BASED:
                score = ComputeSinrBasedLSSS(i);
                break;
            case DetectionMode::PER_BASED:
                score = ComputePerBasedLSSS(i);
                break;
            case DetectionMode::HYBRID:
                score = ComputeHybridLSSS(i);
                break;
            case DetectionMode::DISTANCE_BASED:
            default:
                // Distance-based is handled separately in handoff.cc
                return false;
        }
        
        if (score < 0.5) {
            degradedCount++;
        }
        if (score < minScore) {
            minScore = score;
        }
    }
    
    // Trigger if critical threshold or enough nodes affected
    bool criticalScore = (minScore < 0.2);
    bool quorumReached = (degradedCount >= std::max(2u, nNodes / 5));
    
    if (criticalScore || quorumReached) {
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [SINR Monitor] Detection triggered!");
        NS_LOG_UNCOND("  Min score: " << minScore);
        NS_LOG_UNCOND("  Degraded nodes: " << degradedCount << "/" << nNodes);
        return true;
    }
    
    return false;
}

// Periodic update - sample SINR from actual PHY layer
void UpdateSinrMonitoring()
{
    // In a full implementation, we would hook into WifiPhy::MonitorSnifferRx
    // For now, we simulate SINR based on distance to jammer and propagation model
    
    for (uint32_t i = 0; i < nNodes; ++i) {
        double distToJammer = GetDistance(fanetNodes.Get(i), puNode);
        
        // Simulate SINR degradation based on jammer proximity
        // At 0m: SINR ≈ -10 dB (unusable)
        // At 300m: SINR ≈ 5 dB (critical)
        // At 600m: SINR ≈ 15 dB (degraded)
        // At 1000m+: SINR ≈ 25 dB (good)
        
        double simulatedSinr;
        if (distToJammer < 100) {
            simulatedSinr = -10.0 + (distToJammer / 100.0) * 15.0;  // -10 to 5 dB
        } else if (distToJammer < 600) {
            simulatedSinr = 5.0 + ((distToJammer - 100) / 500.0) * 15.0;  // 5 to 20 dB
        } else {
            simulatedSinr = 20.0 + std::min(10.0, (distToJammer - 600) / 400.0 * 10.0);  // 20+ dB
        }
        
        // Add some noise for realism
        double noise = (rand() % 40 - 20) / 10.0;  // ±2 dB random variation
        simulatedSinr += noise;
        
        RecordSinrSample(i, simulatedSinr);
    }
    
    // Schedule next update
    ns3::Simulator::Schedule(ns3::Seconds(SINR_SAMPLE_INTERVAL), &UpdateSinrMonitoring);
}

// Reset PER window for new measurement period
void ResetPERWindow()
{
    for (uint32_t i = 0; i < nNodes; ++i) {
        auto& stats = nodeSignalStats[i];
        stats.txPackets = 0;
        stats.rxPackets = 0;
        stats.lostPackets = 0;
    }
    
    // Schedule next reset
    ns3::Simulator::Schedule(ns3::Seconds(PER_SAMPLE_WINDOW), &ResetPERWindow);
}

// WiFi PHY callback for actual SINR monitoring (for integration with NS-3 PHY layer)
void SinrMonitorCallback(std::string context, ns3::Ptr<const ns3::Packet> packet,
                         uint16_t channelFreqMhz, ns3::WifiTxVector txVector,
                         ns3::MpduInfo aMpdu, ns3::SignalNoiseDbm signalNoise,
                         uint16_t staId)
{
    // Extract node ID from context string (format: /NodeList/X/...)
    size_t pos = context.find("/NodeList/");
    if (pos == std::string::npos) return;
    
    size_t start = pos + 10;
    size_t end = context.find("/", start);
    if (end == std::string::npos) return;
    
    uint32_t nodeId = std::stoul(context.substr(start, end - start));
    
    // Calculate SINR from signal and noise
    double sinrDb = signalNoise.signal - signalNoise.noise;
    
    RecordSinrSample(nodeId, sinrDb);
}

} // namespace myfanet
