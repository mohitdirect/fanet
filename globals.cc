/*
 * globals.cc - Global Variable Definitions
 * 
 * This file contains the definitions for all extern variables
 * declared in config.h and types.h
 */

#include "config.h"
#include "types.h"
#include "ns3/mobility-module.h"
#include <cmath>
#include <algorithm>

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
bool handoffDisabled = false;  // Set true for protocols without anti-jamming (mp-gpsr, g-olsr)
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

// --- Super-Cluster Data (Hierarchical) ---
std::vector<SuperClusterInfo> superClusters;
std::map<uint32_t, uint32_t> clusterToSuperCluster;
std::map<uint32_t, uint32_t> nodeToSuperCluster;
uint32_t numSuperClusters = MIN_SUPER_CLUSTERS;

// --- Global Node Containers ---
ns3::NodeContainer fanetNodes;
ns3::Ptr<ns3::Node> puNode;
ns3::ApplicationContainer jammerApps;

// --- Velocity History for Φ6 ---
std::map<uint32_t, VelocityHistory> velocityHistories;

// --- Spatial Grid Instance ---
SpatialGrid spatialGrid;

// --- Cluster Stability Tracking ---
double lastClusterScoreSum = 0.0;

// --- Jammer Mode ---
JammerMode currentJammerMode = JammerMode::STATIC_SCRIPTED;

// --- Control Overhead Tracking ---
uint64_t controlBytesOLSR = 0;
uint64_t controlBytesLMSS = 0;
uint64_t controlBytesCluster = 0;
uint64_t controlBytesHandoff = 0;

// === SpatialGrid Implementation ===
std::pair<int, int> SpatialGrid::GetCell(const ns3::Vector& pos) const {
    int x = static_cast<int>(pos.x / SPATIAL_GRID_CELL_SIZE);
    int y = static_cast<int>(pos.y / SPATIAL_GRID_CELL_SIZE);
    return std::make_pair(x, y);
}

void SpatialGrid::Update(uint32_t nodeId, const ns3::Vector& pos) {
    // Remove from old cell if exists
    auto oldIt = nodePositions_.find(nodeId);
    if (oldIt != nodePositions_.end()) {
        auto oldCell = GetCell(oldIt->second);
        auto& oldCellNodes = grid_[oldCell];
        oldCellNodes.erase(
            std::remove(oldCellNodes.begin(), oldCellNodes.end(), nodeId),
            oldCellNodes.end());
    }
    
    // Add to new cell
    nodePositions_[nodeId] = pos;
    auto newCell = GetCell(pos);
    grid_[newCell].push_back(nodeId);
}

std::vector<uint32_t> SpatialGrid::GetCandidateNeighbors(uint32_t nodeId, double range) const {
    std::vector<uint32_t> neighbors;
    
    auto it = nodePositions_.find(nodeId);
    if (it == nodePositions_.end()) return neighbors;
    
    auto myCell = GetCell(it->second);
    int cellRadius = static_cast<int>(std::ceil(range / SPATIAL_GRID_CELL_SIZE)) + 1;
    
    // Search nearby cells
    for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
        for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
            auto cell = std::make_pair(myCell.first + dx, myCell.second + dy);
            auto cellIt = grid_.find(cell);
            if (cellIt != grid_.end()) {
                for (uint32_t candidateId : cellIt->second) {
                    if (candidateId != nodeId) {
                        neighbors.push_back(candidateId);
                    }
                }
            }
        }
    }
    
    return neighbors;
}

void SpatialGrid::Rebuild() {
    grid_.clear();
    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::MobilityModel> mm = fanetNodes.Get(i)->GetObject<ns3::MobilityModel>();
        if (mm) {
            Update(i, mm->GetPosition());
        }
    }
    needsRebuild_ = false;
}

} // namespace myfanet
