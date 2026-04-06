/*
 * load_balancer.h - Traffic Load Balancing for Scalability
 * 
 * Prevents cluster head bottlenecks by:
 * 1. Direct transmission when destination is nearby
 * 2. Backup relay selection when CH is congested
 * 3. Rate limiting at overloaded nodes
 */

#ifndef MYFANET_LOAD_BALANCER_H
#define MYFANET_LOAD_BALANCER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <map>
#include <vector>

namespace myfanet {

// Node load tracking
struct NodeLoad {
    uint32_t nodeId = 0;
    double queueOccupancy = 0.0;      // 0-1 fraction
    uint32_t packetsQueued = 0;
    uint32_t packetsForwarded = 0;
    ns3::Time lastUpdate;
    
    bool isOverloaded() const { return queueOccupancy > 0.7; }
};

// Relay candidate info
struct RelayCandidate {
    uint32_t nodeId;
    double distance;        // Distance to destination
    double lmssScore;       // LMSS score (higher = better)
    double load;            // Current load (lower = better)
    double combinedScore;   // Weighted combination
};

// Load Balancer Singleton
class LoadBalancer {
public:
    static LoadBalancer& Instance();
    
    // === Initialization ===
    void Initialize(uint32_t numNodes);
    
    // === Load Tracking ===
    void UpdateNodeLoad(uint32_t nodeId, double queueOccupancy, uint32_t packetsQueued);
    void RecordPacketForwarded(uint32_t nodeId);
    const NodeLoad& GetNodeLoad(uint32_t nodeId) const;
    bool IsNodeOverloaded(uint32_t nodeId) const;
    
    // === Routing Decisions ===
    // Returns optimal next hop: -1 = direct, nodeId = relay
    int32_t GetOptimalNextHop(uint32_t srcNode, uint32_t dstNode, uint32_t clusterHead);
    
    // Check if direct transmission is feasible
    bool CanTransmitDirect(uint32_t srcNode, uint32_t dstNode);
    
    // Get backup relay when CH is overloaded
    uint32_t GetBackupRelay(uint32_t srcNode, uint32_t dstNode, uint32_t overloadedCH);
    
    // === Rate Control ===
    double GetRecommendedRate(uint32_t nodeId);  // Returns rate multiplier (0-1)
    bool ShouldThrottle(uint32_t nodeId);
    
    // === Statistics ===
    void PrintLoadSummary() const;
    uint32_t GetOverloadedNodeCount() const;
    double GetAverageLoad() const;
    
private:
    LoadBalancer() = default;
    ~LoadBalancer() = default;
    LoadBalancer(const LoadBalancer&) = delete;
    LoadBalancer& operator=(const LoadBalancer&) = delete;
    
    // === Configuration ===
    static constexpr double DIRECT_TX_RANGE = 300.0;      // Max range for direct transmission
    static constexpr double OVERLOAD_THRESHOLD = 0.7;     // Queue occupancy threshold
    static constexpr double THROTTLE_THRESHOLD = 0.85;    // Start throttling above this
    static constexpr uint32_t MAX_DIRECT_HOPS = 2;        // Max hops for "direct" path
    
    // === State ===
    std::map<uint32_t, NodeLoad> nodeLoads_;
    uint32_t numNodes_ = 0;
    bool initialized_ = false;
    
    // === Internal helpers ===
    std::vector<RelayCandidate> FindRelayCandidates(uint32_t srcNode, uint32_t dstNode);
    double ComputeRelayScore(const RelayCandidate& candidate);
    
    static LoadBalancer* instance_;
};

} // namespace myfanet

#endif // MYFANET_LOAD_BALANCER_H
