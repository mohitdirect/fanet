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
    uint32_t breakCount = 0;           // Number of link breaks (for stability tracking)
    ns3::Time lastBreakTime;           // When the link last broke
    bool wasConnected = false;         // Previous connection state
};

// --- Velocity History for Φ6 Predictability ---
struct VelocityHistory {
    std::vector<ns3::Vector> samples;  // Recent velocity samples
    ns3::Time lastSampleTime;          // Time of last sample
    double variance = 0.0;             // Computed variance (cached)
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
    uint32_t superClusterId = 0;  // Parent super-cluster ID
};

// --- Super-Cluster for Hierarchical Routing ---
struct SuperClusterInfo {
    uint32_t superClusterId;              // Super-cluster ID
    uint32_t superClusterHeadId;          // Node ID of super-cluster head (elected from CHs)
    std::vector<uint32_t> memberClusters; // Local cluster IDs in this super-cluster
    std::vector<uint32_t> memberNodes;    // All node IDs (flattened)
    ns3::Vector centroid;                 // Geographic center
    double avgScore;                      // Average combined score
};

// --- Global Data Containers ---
extern std::vector<NodeMetrics> nodeScores;
extern std::map<std::pair<uint32_t, uint32_t>, LinkHistory> linkHistories;
extern std::vector<AircraftCommPair> aircraftPairs;

// --- Cluster Data ---
extern std::vector<ClusterInfo> clusters;           // All clusters
extern std::map<uint32_t, uint32_t> nodeToCluster;  // Node ID -> Cluster ID mapping
extern uint32_t numClusters;                        // Number of clusters

// --- Super-Cluster Data (Hierarchical) ---
extern std::vector<SuperClusterInfo> superClusters;     // All super-clusters
extern std::map<uint32_t, uint32_t> clusterToSuperCluster;  // Cluster ID -> Super-cluster ID
extern std::map<uint32_t, uint32_t> nodeToSuperCluster;     // Node ID -> Super-cluster ID
extern uint32_t numSuperClusters;                       // Number of super-clusters

// --- Global Node Containers ---
extern ns3::NodeContainer fanetNodes;
extern ns3::Ptr<ns3::Node> puNode;
extern ns3::ApplicationContainer jammerApps;

// --- Velocity History Container ---
extern std::map<uint32_t, VelocityHistory> velocityHistories;

// --- Spatial Grid for O(n) Neighbor Discovery ---
class SpatialGrid {
public:
    // Update a node's position in the grid
    void Update(uint32_t nodeId, const ns3::Vector& pos);
    
    // Get all potential neighbors within range (cells overlap for safety)
    std::vector<uint32_t> GetCandidateNeighbors(uint32_t nodeId, double range) const;
    
    // Clear and rebuild the entire grid
    void Rebuild();
    
    // Check if grid needs rebuilding (positions have changed significantly)
    bool NeedsRebuild() const { return needsRebuild_; }
    void MarkNeedsRebuild() { needsRebuild_ = true; }

private:
    std::pair<int, int> GetCell(const ns3::Vector& pos) const;
    std::map<std::pair<int, int>, std::vector<uint32_t>> grid_;
    std::map<uint32_t, ns3::Vector> nodePositions_;
    bool needsRebuild_ = true;
};

extern SpatialGrid spatialGrid;

// --- Cluster Stability Tracking ---
extern double lastClusterScoreSum;  // Sum of combined scores when clusters last formed

} // namespace myfanet

#endif // MYFANET_TYPES_H
