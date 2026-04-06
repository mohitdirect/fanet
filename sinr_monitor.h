/*
 * sinr_monitor.h - SINR and PER Monitoring for Realistic Jammer Detection
 * 
 * This module provides Signal-to-Interference-plus-Noise Ratio (SINR) based
 * detection instead of omniscient distance-based detection.
 */

#ifndef MYFANET_SINR_MONITOR_H
#define MYFANET_SINR_MONITOR_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include <vector>
#include <map>

namespace myfanet {

// Per-node SINR/PER statistics
struct NodeSignalStats {
    std::vector<double> sinrHistory;      // Recent SINR samples (dB)
    double avgSinr = 20.0;                // Running average SINR
    double minSinr = 20.0;                // Minimum SINR observed
    
    uint32_t txPackets = 0;               // Packets transmitted in window
    uint32_t rxPackets = 0;               // Packets received in window
    uint32_t lostPackets = 0;             // Packets lost in window
    double packetErrorRate = 0.0;         // Current PER
    
    ns3::Time lastSampleTime;             // When last sampled
    bool isDegraded = false;              // Currently experiencing interference
};

// Global SINR/PER data
extern std::map<uint32_t, NodeSignalStats> nodeSignalStats;

// === API Functions ===

// Initialize SINR monitoring for all nodes
void InitializeSinrMonitoring();

// Sample SINR for a specific node (call from PHY layer)
void RecordSinrSample(uint32_t nodeId, double sinrDb);

// Record packet transmission/reception for PER calculation
void RecordPacketTx(uint32_t nodeId);
void RecordPacketRx(uint32_t nodeId);
void RecordPacketLoss(uint32_t nodeId);

// Get current SINR-based LSSS score (0-1)
double ComputeSinrBasedLSSS(uint32_t nodeId);

// Get current PER-based detection score (0-1)
double ComputePerBasedLSSS(uint32_t nodeId);

// Combined hybrid detection score
double ComputeHybridLSSS(uint32_t nodeId);

// Check if handoff should be triggered (based on detection mode)
bool ShouldTriggerHandoff();

// Periodic update function (call from main loop)
void UpdateSinrMonitoring();

// Reset statistics for new simulation window
void ResetPERWindow();

// WiFi PHY callback for SINR monitoring
void SinrMonitorCallback(std::string context, ns3::Ptr<const ns3::Packet> packet,
                         uint16_t channelFreqMhz, ns3::WifiTxVector txVector,
                         ns3::MpduInfo aMpdu, ns3::SignalNoiseDbm signalNoise,
                         uint16_t staId);

} // namespace myfanet

#endif // MYFANET_SINR_MONITOR_H
