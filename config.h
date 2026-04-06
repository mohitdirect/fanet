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
extern bool handoffDisabled;  // Disable handoff for protocols without anti-jamming
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

// === Jammer Position Constants (formerly magic numbers) ===
constexpr double JAMMER_START_POSITION_X = 1600.0;  // Initial jammer X position
constexpr double JAMMER_START_POSITION_Y = 500.0;   // Initial jammer Y position
constexpr double JAMMER_START_POSITION_Z = 75.0;    // Initial jammer Z position
constexpr double JAMMER_ISOLATION_X = 5000.0;       // Where jammer is moved on handoff
constexpr double JAMMER_ISOLATION_Y = 5000.0;
constexpr double JAMMER_ISOLATION_Z = 1000.0;
constexpr double JAMMER_APPROACH_VELOCITY = -15.0;  // Velocity toward network
constexpr double JAMMER_RETREAT_VELOCITY = 15.0;    // Velocity away from network

// === Communication and Detection Ranges ===
constexpr double DEFAULT_COMM_RANGE = 600.0;        // Default communication range
constexpr double LSSS_BASE_RANGE = 300.0;           // Base distance for LSSS calculation
constexpr double LSSS_SCALE_RANGE = 1000.0;         // Scale factor for LSSS

// === Velocity History for Φ6 Predictability ===
constexpr uint32_t VELOCITY_HISTORY_SIZE = 10;      // Number of samples to track
constexpr double VELOCITY_SAMPLE_INTERVAL = 0.5;    // Seconds between samples

// === Spatial Grid for Efficient Neighbor Discovery ===
constexpr double SPATIAL_GRID_CELL_SIZE = 200.0;    // Grid cell size in meters
constexpr uint32_t SPATIAL_GRID_SIZE = 10;          // Grid dimensions (10x10)

// === Cluster Stability ===
constexpr double CLUSTER_STABILITY_THRESHOLD = 0.1; // Min score change to re-form clusters
constexpr double CLUSTER_UPDATE_INTERVAL = 5.0;     // Seconds between cluster updates

// === Hierarchical Clustering (Super-Clusters) ===
constexpr uint32_t NODES_PER_SUPER_CLUSTER = 25;    // Target nodes per super-cluster
constexpr uint32_t MIN_SUPER_CLUSTERS = 2;          // Minimum super-clusters
constexpr uint32_t MAX_SUPER_CLUSTERS = 8;          // Maximum super-clusters
constexpr double SUPER_CLUSTER_RANGE = 800.0;       // Max range for super-cluster membership
constexpr double SUPER_CH_SCORE_BONUS = 0.1;        // Bonus for existing super-CH

// === SINR-Based Detection (Publication Quality) ===
constexpr double SINR_GOOD_THRESHOLD = 20.0;        // dB - excellent link quality
constexpr double SINR_DEGRADED_THRESHOLD = 10.0;    // dB - degraded but usable
constexpr double SINR_CRITICAL_THRESHOLD = 5.0;     // dB - critical, handoff needed
constexpr double SINR_SAMPLE_INTERVAL = 0.2;        // Seconds between SINR samples
constexpr uint32_t SINR_HISTORY_SIZE = 10;          // Number of samples to average
constexpr double SINR_ALERT_RATIO = 0.5;            // Ratio of nodes with degraded SINR to trigger alert

// === Packet Error Rate (PER) Monitoring ===
constexpr double PER_NORMAL_THRESHOLD = 0.05;       // 5% PER is normal
constexpr double PER_DEGRADED_THRESHOLD = 0.15;     // 15% PER indicates interference
constexpr double PER_CRITICAL_THRESHOLD = 0.30;     // 30% PER triggers handoff
constexpr double PER_SAMPLE_WINDOW = 2.0;           // Seconds to measure PER

// === Realistic Handoff Delay Parameters ===
constexpr double HANDOFF_ANNOUNCEMENT_DELAY = 0.1;  // 100ms for broadcast announcement
constexpr double HANDOFF_QUIET_PERIOD = 0.05;       // 50ms quiet period
constexpr double HANDOFF_SWITCH_DELAY = 0.0;        // Instant channel switch (in simulation)
constexpr double HANDOFF_ROUTE_CONVERGENCE = 3.0;   // 3s for OLSR to converge
constexpr double HANDOFF_FAILURE_PROBABILITY = 0.05; // 5% of nodes fail to switch

// === Detection Mode Selection ===
enum class DetectionMode {
    DISTANCE_BASED,   // Original omniscient mode (for comparison)
    SINR_BASED,       // Realistic SINR-based detection
    PER_BASED,        // Packet error rate based
    HYBRID            // Combined SINR + PER
};
extern DetectionMode currentDetectionMode;

// === Jammer Mode Selection (Publication Quality) ===
enum class JammerMode {
    STATIC_SCRIPTED,  // Original: scripted approach/park/retreat
    MOBILE_RANDOM,    // Mobile jammer with Gauss-Markov movement
    SMART_TRACKING,   // Smart jammer that targets cluster centroid
    INTERMITTENT      // On/off jamming with duty cycle
};
extern JammerMode currentJammerMode;

// === Mobile Jammer Parameters ===
constexpr double MOBILE_JAMMER_MEAN_VELOCITY = 15.0;  // m/s average speed
constexpr double MOBILE_JAMMER_ALPHA = 0.8;           // Gauss-Markov alpha
constexpr double MOBILE_JAMMER_UPDATE_INTERVAL = 0.5; // Seconds

// === Smart Jammer Parameters ===
constexpr double SMART_JAMMER_TRACKING_RANGE = 800.0; // Range to track cluster
constexpr double SMART_JAMMER_VELOCITY = 12.0;        // Pursuit velocity

// === Intermittent Jammer Parameters ===
constexpr double INTERMITTENT_ON_TIME = 5.0;          // Active time (seconds)
constexpr double INTERMITTENT_OFF_TIME = 3.0;         // Inactive time (seconds)

// === Control Overhead Tracking ===
extern uint64_t controlBytesOLSR;      // OLSR hello/TC messages
extern uint64_t controlBytesLMSS;      // LMSS monitoring overhead
extern uint64_t controlBytesCluster;   // Cluster maintenance overhead
extern uint64_t controlBytesHandoff;   // Handoff coordination overhead

// === Handoff Parameters (formerly magic numbers) ===
constexpr uint32_t HANDOFF_MIN_PACKET_LOSS = 10;      // Minimum packets lost during handoff
constexpr uint32_t HANDOFF_MAX_PACKET_LOSS = 30;      // Maximum packets lost during handoff
constexpr double HANDOFF_VERIFY_DISTANCE = 3000.0;    // Distance to consider jammer isolated
constexpr double HANDOFF_VERIFY_DELAY = 0.5;          // Delay before verification (seconds)

// === Multi-Seed Testing ===
constexpr uint32_t TEST_SEEDS[] = {3, 7, 42, 123, 999};
constexpr uint32_t NUM_TEST_SEEDS = 5;

} // namespace myfanet

#endif // MYFANET_CONFIG_H
