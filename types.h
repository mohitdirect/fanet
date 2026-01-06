/*
 * types.h - Data Structures for PGH-LMSS FANET Simulation
 */

#ifndef MYFANET_TYPES_H
#define MYFANET_TYPES_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <map>
#include <vector>

namespace myfanet {

// --- PGH-LMSS Node Metrics ---
struct NodeMetrics {
    double lmss;
    double lsss;
    double combinedScore;
};

// --- Link History for LMSS Calculation ---
struct LinkHistory {
    ns3::Time linkStartTime;
    double lastDistance;
    ns3::Vector lastVelocity;
};

// --- Aircraft Communication Configuration ---
struct AircraftCommPair {
    uint32_t sender;
    std::vector<uint32_t> receivers;
};

// --- Cluster Information for CH-Based Routing ---
struct ClusterInfo {
    uint32_t clusterId;           // Cluster identifier (same as CH node ID)
    uint32_t clusterHeadId;       // Node ID of cluster head
    std::vector<uint32_t> members; // Member node IDs
    double avgLmss;               // Average LMSS of cluster
};

// --- Global Data Containers ---
extern std::vector<NodeMetrics> nodeScores;
extern std::map<std::pair<uint32_t, uint32_t>, LinkHistory> linkHistories;
extern std::vector<AircraftCommPair> aircraftPairs;

// --- Cluster Data ---
extern std::vector<ClusterInfo> clusters;           // All clusters
extern std::map<uint32_t, uint32_t> nodeToCluster;  // Node ID -> Cluster ID mapping
extern uint32_t numClusters;                        // Number of clusters

// --- Global Node Containers ---
extern ns3::NodeContainer fanetNodes;
extern ns3::Ptr<ns3::Node> puNode;
extern ns3::ApplicationContainer jammerApps;

} // namespace myfanet

#endif // MYFANET_TYPES_H
