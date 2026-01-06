/*
 * globals.cc - Global Variable Definitions
 * 
 * This file contains the definitions for all extern variables
 * declared in config.h and types.h
 */

#include "config.h"
#include "types.h"

namespace myfanet {

// --- Simulation Parameters ---
uint32_t nNodes = 30;
double simTime = 60.0;
double pghPredictionRange = 1500.0;
double jammerStartTime = 17.5;

// --- Traffic Mode ---
TrafficPattern trafficMode = AIRCRAFT_ACK;

// --- Comparison Configuration ---
std::string protocol = "pgh";
bool actionTaken = false;
ns3::Time detectionTime = ns3::Seconds(0.0);

// === Phase Tracking for Time-Segmented Metrics ===
SimulationPhase currentPhase = PRE_JAMMING;
ns3::Time jammingStartTimeActual = ns3::Seconds(0.0);
ns3::Time recoveryStartTime = ns3::Seconds(0.0);
double preJammingBytes = 0.0;
double duringJammingBytes = 0.0;
double postRecoveryBytes = 0.0;
double preJammingPackets = 0.0;
double duringJammingPackets = 0.0;
double postRecoveryPackets = 0.0;
double recoveryDelaySeconds = 5.0;  // Wait 5s after handoff before measuring recovery

// === CR mode parameters ===
double crEdDetectRange = 300.0;
double crScanInterval = 0.5;
double crScanRange = 350.0;
double crCoopRange = 450.0;
uint32_t crCoopQuorum = 3;

// === Enhanced PGH-LMSS Parameters ===
double lsssHandoffThreshold = 0.4;
double combinedScoreThreshold = 0.35;
double criticalLsssThreshold = 0.2;
uint32_t affectedNodesQuorum = 2;

double lmssWeightNormal = 0.7;
double lmssWeightThreat = 0.3;
double lsssWeightNormal = 0.3;
double lsssWeightThreat = 0.7;

// === CH Election Stability ===
uint32_t currentClusterHeadId = 999;
double chChangeThreshold = 0.05;
ns3::Time minChHoldTime = ns3::Seconds(5.0);
ns3::Time lastChChangeTime = ns3::Seconds(0.0);
double currentClusterHeadScore = -1.0;

// --- Global Data Containers ---
std::vector<NodeMetrics> nodeScores;
std::map<std::pair<uint32_t, uint32_t>, LinkHistory> linkHistories;
std::vector<AircraftCommPair> aircraftPairs;

// --- Cluster Data ---
std::vector<ClusterInfo> clusters;
std::map<uint32_t, uint32_t> nodeToCluster;
uint32_t numClusters = DEFAULT_NUM_CLUSTERS;

// --- Global Node Containers ---
ns3::NodeContainer fanetNodes;
ns3::Ptr<ns3::Node> puNode;
ns3::ApplicationContainer jammerApps;

} // namespace myfanet
