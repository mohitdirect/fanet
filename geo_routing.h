/*
 * geo_routing.h - LMSS-Aware Geographic Routing
 * 
 * Provides geographic (position-based) routing as an alternative to OLSR
 * for improved scalability. Uses LMSS scores to weight neighbor selection.
 * 
 * Features:
 * - Greedy geographic forwarding (toward destination)
 * - LMSS-weighted neighbor selection (prefer stable links)
 * - Perimeter routing for recovery from local minima
 * - Integration with hierarchical clustering
 */

#ifndef MYFANET_GEO_ROUTING_H
#define MYFANET_GEO_ROUTING_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <vector>
#include <map>

namespace myfanet {

// Neighbor information for geographic routing
struct GeoNeighbor {
    uint32_t nodeId;
    ns3::Vector position;
    double distanceToDest;    // Distance to destination
    double lmssScore;         // Link quality score
    double combinedScore;     // Weighted combination for selection
    bool isClusterHead;       // Is this neighbor a CH?
    bool isSuperClusterHead;  // Is this neighbor a super-CH?
};

// Geographic routing modes
enum class GeoRoutingMode {
    GREEDY,       // Always pick neighbor closest to destination
    LMSS_WEIGHTED, // Weight by LMSS score (default)
    PERIMETER     // Recovery mode when greedy fails
};

// Geographic Router Singleton
class GeoRouter {
public:
    static GeoRouter& Instance();
    
    // === Initialization ===
    void Initialize(uint32_t numNodes);
    
    // === Neighbor Management ===
    void UpdateNeighborInfo(uint32_t nodeId);
    std::vector<GeoNeighbor> GetNeighbors(uint32_t nodeId) const;
    
    // === Routing Decisions ===
    // Get next hop using geographic routing
    // Returns UINT32_MAX if no valid next hop
    uint32_t GetNextHop(uint32_t srcNode, uint32_t dstNode);
    
    // Get next hop with specific mode
    uint32_t GetNextHopWithMode(uint32_t srcNode, uint32_t dstNode, GeoRoutingMode mode);
    
    // Check if destination is directly reachable
    bool IsDirectlyReachable(uint32_t srcNode, uint32_t dstNode) const;
    
    // === Perimeter Routing (void recovery) ===
    uint32_t GetPerimeterNextHop(uint32_t srcNode, uint32_t dstNode);
    bool IsInVoid(uint32_t srcNode, uint32_t dstNode) const;
    
    // === Hierarchical Routing Integration ===
    // Route through cluster/super-cluster hierarchy
    uint32_t GetHierarchicalNextHop(uint32_t srcNode, uint32_t dstNode);
    
    // === Configuration ===
    void SetMode(GeoRoutingMode mode) { currentMode_ = mode; }
    GeoRoutingMode GetMode() const { return currentMode_; }
    
    void SetLmssWeight(double weight) { lmssWeight_ = weight; }
    double GetLmssWeight() const { return lmssWeight_; }
    
    // === Statistics ===
    void PrintRoutingSummary() const;
    uint32_t GetGreedySuccessCount() const { return greedySuccessCount_; }
    uint32_t GetPerimeterFallbackCount() const { return perimeterFallbackCount_; }
    
private:
    GeoRouter() = default;
    ~GeoRouter() = default;
    GeoRouter(const GeoRouter&) = delete;
    GeoRouter& operator=(const GeoRouter&) = delete;
    
    // === Configuration ===
    static constexpr double DEFAULT_LMSS_WEIGHT = 0.4;  // Weight for LMSS in scoring
    static constexpr double DISTANCE_WEIGHT = 0.6;       // Weight for distance progress
    static constexpr double COMM_RANGE_FACTOR = 0.9;     // Safety margin for range
    
    // === State ===
    std::map<uint32_t, std::vector<GeoNeighbor>> neighborCache_;
    GeoRoutingMode currentMode_ = GeoRoutingMode::LMSS_WEIGHTED;
    double lmssWeight_ = DEFAULT_LMSS_WEIGHT;
    uint32_t numNodes_ = 0;
    bool initialized_ = false;
    
    // === Statistics ===
    mutable uint32_t greedySuccessCount_ = 0;
    mutable uint32_t perimeterFallbackCount_ = 0;
    mutable uint32_t hierarchicalRouteCount_ = 0;
    
    // === Internal helpers ===
    double ComputeNeighborScore(const GeoNeighbor& neighbor, double srcToDstDist) const;
    std::vector<GeoNeighbor> FindNeighborsInRange(uint32_t nodeId, double range) const;
    
    static GeoRouter* instance_;
};

} // namespace myfanet

#endif // MYFANET_GEO_ROUTING_H
