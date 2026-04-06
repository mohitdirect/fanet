/*
 * load_balancer.cc - Traffic Load Balancing Implementation
 * 
 * Implements load balancing to prevent cluster head bottlenecks
 * and improve scalability at 50+ nodes.
 */

#include "load_balancer.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "lmss.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>

NS_LOG_COMPONENT_DEFINE("LoadBalancer");

namespace myfanet {

// Singleton instance
LoadBalancer* LoadBalancer::instance_ = nullptr;

LoadBalancer& LoadBalancer::Instance() {
    if (!instance_) {
        instance_ = new LoadBalancer();
    }
    return *instance_;
}

void LoadBalancer::Initialize(uint32_t numNodes) {
    numNodes_ = numNodes;
    nodeLoads_.clear();
    
    // Initialize load tracking for all nodes
    for (uint32_t i = 0; i < numNodes; ++i) {
        NodeLoad load;
        load.nodeId = i;
        load.queueOccupancy = 0.0;
        load.packetsQueued = 0;
        load.packetsForwarded = 0;
        load.lastUpdate = ns3::Simulator::Now();
        nodeLoads_[i] = load;
    }
    
    initialized_ = true;
    NS_LOG_UNCOND("[LoadBalancer] Initialized for " << numNodes << " nodes");
}

void LoadBalancer::UpdateNodeLoad(uint32_t nodeId, double queueOccupancy, uint32_t packetsQueued) {
    if (nodeId >= numNodes_) return;
    
    auto& load = nodeLoads_[nodeId];
    load.queueOccupancy = queueOccupancy;
    load.packetsQueued = packetsQueued;
    load.lastUpdate = ns3::Simulator::Now();
}

void LoadBalancer::RecordPacketForwarded(uint32_t nodeId) {
    if (nodeId >= numNodes_) return;
    nodeLoads_[nodeId].packetsForwarded++;
}

const NodeLoad& LoadBalancer::GetNodeLoad(uint32_t nodeId) const {
    static NodeLoad defaultLoad;
    auto it = nodeLoads_.find(nodeId);
    return (it != nodeLoads_.end()) ? it->second : defaultLoad;
}

bool LoadBalancer::IsNodeOverloaded(uint32_t nodeId) const {
    auto it = nodeLoads_.find(nodeId);
    if (it == nodeLoads_.end()) return false;
    return it->second.isOverloaded();
}

bool LoadBalancer::CanTransmitDirect(uint32_t srcNode, uint32_t dstNode) {
    // Check if nodes are within direct transmission range
    double distance = GetDistance(fanetNodes.Get(srcNode), fanetNodes.Get(dstNode));
    
    // Also check LMSS score for link quality
    double lmss = ComputeEnhancedLMSS(srcNode, DEFAULT_COMM_RANGE);
    
    // Allow direct if close AND good link quality
    return (distance < DIRECT_TX_RANGE) && (lmss > 0.5);
}

int32_t LoadBalancer::GetOptimalNextHop(uint32_t srcNode, uint32_t dstNode, uint32_t clusterHead) {
    // Option 1: Direct transmission if possible
    if (CanTransmitDirect(srcNode, dstNode)) {
        return -1;  // -1 indicates direct transmission
    }
    
    // Option 2: Use CH if not overloaded
    if (!IsNodeOverloaded(clusterHead)) {
        return static_cast<int32_t>(clusterHead);
    }
    
    // Option 3: Find backup relay
    uint32_t backup = GetBackupRelay(srcNode, dstNode, clusterHead);
    if (backup != UINT32_MAX) {
        return static_cast<int32_t>(backup);
    }
    
    // Fallback to CH anyway
    return static_cast<int32_t>(clusterHead);
}

uint32_t LoadBalancer::GetBackupRelay(uint32_t srcNode, uint32_t dstNode, uint32_t overloadedCH) {
    auto candidates = FindRelayCandidates(srcNode, dstNode);
    
    // Remove overloaded CH from candidates
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
            [overloadedCH](const RelayCandidate& c) { 
                return c.nodeId == overloadedCH; 
            }),
        candidates.end()
    );
    
    if (candidates.empty()) {
        return UINT32_MAX;
    }
    
    // Sort by combined score (higher = better)
    std::sort(candidates.begin(), candidates.end(),
        [](const RelayCandidate& a, const RelayCandidate& b) {
            return a.combinedScore > b.combinedScore;
        });
    
    return candidates[0].nodeId;
}

std::vector<RelayCandidate> LoadBalancer::FindRelayCandidates(uint32_t srcNode, uint32_t dstNode) {
    std::vector<RelayCandidate> candidates;
    
    ns3::Ptr<ns3::Node> src = fanetNodes.Get(srcNode);
    ns3::Ptr<ns3::Node> dst = fanetNodes.Get(dstNode);
    double srcToDst = GetDistance(src, dst);
    
    for (uint32_t i = 0; i < nNodes; ++i) {
        if (i == srcNode || i == dstNode) continue;
        
        ns3::Ptr<ns3::Node> relay = fanetNodes.Get(i);
        double srcToRelay = GetDistance(src, relay);
        double relayToDst = GetDistance(relay, dst);
        
        // Only consider nodes that are:
        // 1. Within range of source
        // 2. Closer to destination than source is
        if (srcToRelay < DEFAULT_COMM_RANGE && relayToDst < srcToDst) {
            RelayCandidate candidate;
            candidate.nodeId = i;
            candidate.distance = relayToDst;
            candidate.lmssScore = ComputeEnhancedLMSS(i, DEFAULT_COMM_RANGE);
            candidate.load = GetNodeLoad(i).queueOccupancy;
            candidate.combinedScore = ComputeRelayScore(candidate);
            
            candidates.push_back(candidate);
        }
    }
    
    return candidates;
}

double LoadBalancer::ComputeRelayScore(const RelayCandidate& candidate) {
    // Weighted combination:
    // - Higher LMSS = better (weight 0.4)
    // - Lower load = better (weight 0.4)
    // - Closer to destination = better (weight 0.2)
    
    double lmssComponent = candidate.lmssScore;  // 0-1
    double loadComponent = 1.0 - candidate.load;  // 0-1 (inverted)
    double distComponent = 1.0 - (candidate.distance / DEFAULT_COMM_RANGE);  // 0-1
    distComponent = std::max(0.0, std::min(1.0, distComponent));
    
    return 0.4 * lmssComponent + 0.4 * loadComponent + 0.2 * distComponent;
}

double LoadBalancer::GetRecommendedRate(uint32_t nodeId) {
    auto it = nodeLoads_.find(nodeId);
    if (it == nodeLoads_.end()) return 1.0;
    
    double load = it->second.queueOccupancy;
    
    if (load < OVERLOAD_THRESHOLD) {
        return 1.0;  // Full rate
    } else if (load < THROTTLE_THRESHOLD) {
        // Linear reduction between thresholds
        double reduction = (load - OVERLOAD_THRESHOLD) / (THROTTLE_THRESHOLD - OVERLOAD_THRESHOLD);
        return 1.0 - (0.5 * reduction);  // 50-100%
    } else {
        // Severe throttling
        return 0.3;  // 30% rate
    }
}

bool LoadBalancer::ShouldThrottle(uint32_t nodeId) {
    auto it = nodeLoads_.find(nodeId);
    if (it == nodeLoads_.end()) return false;
    return it->second.queueOccupancy >= THROTTLE_THRESHOLD;
}

uint32_t LoadBalancer::GetOverloadedNodeCount() const {
    uint32_t count = 0;
    for (const auto& kv : nodeLoads_) {
        if (kv.second.isOverloaded()) count++;
    }
    return count;
}

double LoadBalancer::GetAverageLoad() const {
    if (nodeLoads_.empty()) return 0.0;
    
    double sum = 0.0;
    for (const auto& kv : nodeLoads_) {
        sum += kv.second.queueOccupancy;
    }
    return sum / nodeLoads_.size();
}

void LoadBalancer::PrintLoadSummary() const {
    NS_LOG_UNCOND("\n--- Load Balancer Summary ---");
    NS_LOG_UNCOND("Total nodes: " << numNodes_);
    NS_LOG_UNCOND("Overloaded nodes: " << GetOverloadedNodeCount());
    NS_LOG_UNCOND("Average load: " << (GetAverageLoad() * 100) << "%");
    
    // Show most loaded nodes
    std::vector<std::pair<double, uint32_t>> sortedLoads;
    for (const auto& kv : nodeLoads_) {
        sortedLoads.push_back({kv.second.queueOccupancy, kv.first});
    }
    std::sort(sortedLoads.rbegin(), sortedLoads.rend());
    
    NS_LOG_UNCOND("Top 5 loaded nodes:");
    for (size_t i = 0; i < std::min(size_t(5), sortedLoads.size()); ++i) {
        NS_LOG_UNCOND("  Node " << sortedLoads[i].second << ": " 
                      << (sortedLoads[i].first * 100) << "%");
    }
    NS_LOG_UNCOND("-----------------------------\n");
}

} // namespace myfanet
