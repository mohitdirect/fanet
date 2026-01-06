/*
 * config.h - Simulation Parameters and Constants
 * FANET Protocol Comparison: PGH-LMSS vs AODV vs OLSR
 */

#ifndef MYFANET_CONFIG_H
#define MYFANET_CONFIG_H

#include "ns3/core-module.h"
#include <string>
#include <vector>

namespace myfanet {

// --- Simulation Parameters ---
extern uint32_t nNodes;
extern double simTime;
extern double pghPredictionRange;
extern double jammerStartTime;

// === Traffic Pattern Configuration ===
enum TrafficPattern {
    ASYMMETRIC,      // 3 flows to 2 receivers
    SYMMETRIC,       // All nodes communicate
    DISTRIBUTED,     // Random sender-receiver pairs
    MANY_TO_ONE,     // Original pattern
    AIRCRAFT_ACK,    // Aircraft with bidirectional ACK
    CLUSTER          // Cluster head-based routing (reduced congestion)
};

extern TrafficPattern trafficMode;

// --- Comparison Configuration ---
extern std::string protocol;
extern bool actionTaken;
extern ns3::Time detectionTime;

// === Phase Tracking for Time-Segmented Metrics ===
enum SimulationPhase {
    PRE_JAMMING,
    DURING_JAMMING,
    POST_RECOVERY
};
extern SimulationPhase currentPhase;
extern ns3::Time jammingStartTimeActual;
extern ns3::Time recoveryStartTime;
extern double preJammingBytes;
extern double duringJammingBytes;
extern double postRecoveryBytes;
extern double preJammingPackets;
extern double duringJammingPackets;
extern double postRecoveryPackets;
extern double recoveryDelaySeconds;

// === CR mode parameters ===
extern double crEdDetectRange;
extern double crScanInterval;
extern double crScanRange;
extern double crCoopRange;
extern uint32_t crCoopQuorum;

// === Enhanced PGH-LMSS Parameters ===
extern double lsssHandoffThreshold;
extern double combinedScoreThreshold;
extern double criticalLsssThreshold;
extern uint32_t affectedNodesQuorum;

extern double lmssWeightNormal;
extern double lmssWeightThreat;
extern double lsssWeightNormal;
extern double lsssWeightThreat;

// === 6-Component Enhanced FANET LMSS Parameters ===
constexpr double LINK_DURATION_WINDOW = 10.0;      // seconds
constexpr double MAX_DISTANCE = 1000.0;            // meters
constexpr double MAX_RELATIVE_VELOCITY = 20.0;     // m/s
constexpr double MAX_ALTITUDE_DIFF = 100.0;        // meters

// Component weights (must sum to 1.0)
constexpr double BETA_1 = 0.15;  // Φ1: Link duration
constexpr double BETA_2 = 0.20;  // Φ2: Distance stability
constexpr double BETA_3 = 0.15;  // Φ3: Relative velocity
constexpr double BETA_4 = 0.15;  // Φ4: Heading alignment
constexpr double BETA_5 = 0.15;  // Φ5: Altitude stability
constexpr double BETA_6 = 0.20;  // Φ6: Flight-path predictability

// === CH Election Stability ===
extern uint32_t currentClusterHeadId;
extern double chChangeThreshold;
extern ns3::Time minChHoldTime;
extern ns3::Time lastChChangeTime;
extern double currentClusterHeadScore;

// === Cluster-Based Routing Parameters ===
constexpr uint32_t DEFAULT_NUM_CLUSTERS = 5;    // Base number of clusters
constexpr double CLUSTER_RANGE = 400.0;         // Max distance to join cluster
constexpr uint32_t NODES_PER_CLUSTER = 6;       // Target nodes per cluster for scaling
constexpr double MIN_TRAFFIC_RATE = 50.0;       // Minimum Kbps per flow
constexpr double MAX_TRAFFIC_RATE = 150.0;      // Maximum Kbps per flow
constexpr double LSSS_ROUTING_THRESHOLD = 0.5;  // Avoid nodes with LSSS below this

} // namespace myfanet

#endif // MYFANET_CONFIG_H
