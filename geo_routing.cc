/*
 * geo_routing.cc - LMSS-Aware Geographic Routing Implementation
 * 
 * Implements geographic forwarding with LMSS weighting for improved
 * scalability in FANETs with 50+ nodes.
 */

#include "geo_routing.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "lmss.h"
#include "handoff.h"
#include "ns3/mobility-module.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>

NS_LOG_COMPONENT_DEFINE("GeoRouting");

namespace myfanet {

// Singleton instance
GeoRouter* GeoRouter::instance_ = nullptr;

GeoRouter& GeoRouter::Instance() {
    if (!instance_) {
        instance_ = new GeoRouter();
    }
    return *instance_;
}

void GeoRouter::Initialize(uint32_t numNodes) {
    numNodes_ = numNodes;
    neighborCache_.clear();
    greedySuccessCount_ = 0;
    perimeterFallbackCount_ = 0;
    hierarchicalRouteCount_ = 0;
    initialized_ = true;
    
    NS_LOG_UNCOND("[GeoRouter] Initialized for " << numNodes << " nodes");
    NS_LOG_UNCOND("[GeoRouter] Mode: LMSS-weighted, LMSS weight=" << lmssWeight_);
}

void GeoRouter::UpdateNeighborInfo(uint32_t nodeId) {
    if (nodeId >= numNodes_) return;
    
    neighborCache_[nodeId] = FindNeighborsInRange(nodeId, DEFAULT_COMM_RANGE * COMM_RANGE_FACTOR);
}

std::vector<GeoNeighbor> GeoRouter::GetNeighbors(uint32_t nodeId) const {
    auto it = neighborCache_.find(nodeId);
    if (it != neighborCache_.end()) {
        return it->second;
    }
    // Return dynamically computed neighbors if not cached
    return FindNeighborsInRange(nodeId, DEFAULT_COMM_RANGE * COMM_RANGE_FACTOR);
}

std::vector<GeoNeighbor> GeoRouter::FindNeighborsInRange(uint32_t nodeId, double range) const {
    std::vector<GeoNeighbor> neighbors;
    
    ns3::Ptr<ns3::MobilityModel> srcMm = fanetNodes.Get(nodeId)->GetObject<ns3::MobilityModel>();
    if (!srcMm) return neighbors;
    ns3::Vector srcPos = srcMm->GetPosition();
    
    for (uint32_t i = 0; i < nNodes; ++i) {
        if (i == nodeId) continue;
        
        ns3::Ptr<ns3::MobilityModel> dstMm = fanetNodes.Get(i)->GetObject<ns3::MobilityModel>();
        if (!dstMm) continue;
        
        ns3::Vector dstPos = dstMm->GetPosition();
        double dist = GetDistance(fanetNodes.Get(nodeId), fanetNodes.Get(i));
        
        if (dist < range) {
            GeoNeighbor neighbor;
            neighbor.nodeId = i;
            neighbor.position = dstPos;
            neighbor.lmssScore = ComputeEnhancedLMSS(i, DEFAULT_COMM_RANGE);
            neighbor.isClusterHead = (GetClusterHead(i) == i);
            neighbor.isSuperClusterHead = (GetSuperClusterHead(i) == i);
            neighbor.distanceToDest = 0;  // Set when routing
            neighbor.combinedScore = 0;   // Computed during routing
            
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

bool GeoRouter::IsDirectlyReachable(uint32_t srcNode, uint32_t dstNode) const {
    double dist = GetDistance(fanetNodes.Get(srcNode), fanetNodes.Get(dstNode));
    return dist < DEFAULT_COMM_RANGE * COMM_RANGE_FACTOR;
}

uint32_t GeoRouter::GetNextHop(uint32_t srcNode, uint32_t dstNode) {
    return GetNextHopWithMode(srcNode, dstNode, currentMode_);
}

uint32_t GeoRouter::GetNextHopWithMode(uint32_t srcNode, uint32_t dstNode, GeoRoutingMode mode) {
    // Direct delivery if in range
    if (IsDirectlyReachable(srcNode, dstNode)) {
        greedySuccessCount_++;
        return dstNode;
    }
    
    // Get destination position
    ns3::Ptr<ns3::MobilityModel> dstMm = fanetNodes.Get(dstNode)->GetObject<ns3::MobilityModel>();
    if (!dstMm) return UINT32_MAX;
    ns3::Vector dstPos = dstMm->GetPosition();
    
    // Get source distance to destination
    double srcToDst = GetDistance(fanetNodes.Get(srcNode), fanetNodes.Get(dstNode));
    
    // Get neighbors
    std::vector<GeoNeighbor> neighbors = GetNeighbors(srcNode);
    if (neighbors.empty()) {
        return UINT32_MAX;  // Isolated node
    }
    
    // Compute scores for each neighbor
    for (auto& neighbor : neighbors) {
        neighbor.distanceToDest = GetDistance(fanetNodes.Get(neighbor.nodeId), fanetNodes.Get(dstNode));
        neighbor.combinedScore = ComputeNeighborScore(neighbor, srcToDst);
    }
    
    // Sort by combined score (higher = better)
    std::sort(neighbors.begin(), neighbors.end(),
        [](const GeoNeighbor& a, const GeoNeighbor& b) {
            return a.combinedScore > b.combinedScore;
        });
    
    // Find best neighbor that makes progress toward destination
    for (const auto& neighbor : neighbors) {
        if (mode == GeoRoutingMode::GREEDY) {
            // Strict greedy: must be closer to destination
            if (neighbor.distanceToDest < srcToDst) {
                greedySuccessCount_++;
                return neighbor.nodeId;
            }
        } else if (mode == GeoRoutingMode::LMSS_WEIGHTED) {
            // LMSS-weighted: consider score, allow slight regression if link is good
            if (neighbor.distanceToDest < srcToDst * 1.1 && neighbor.lmssScore > 0.3) {
                greedySuccessCount_++;
                return neighbor.nodeId;
            }
        }
    }
    
    // Greedy failed - try perimeter routing or hierarchical
    if (mode != GeoRoutingMode::PERIMETER) {
        return GetPerimeterNextHop(srcNode, dstNode);
    }
    
    return UINT32_MAX;
}

double GeoRouter::ComputeNeighborScore(const GeoNeighbor& neighbor, double srcToDstDist) const {
    // Progress toward destination (0-1, higher = closer)
    double progress = 1.0 - (neighbor.distanceToDest / srcToDstDist);
    progress = std::max(0.0, std::min(1.0, progress));
    
    // LMSS score (0-1)
    double lmss = neighbor.lmssScore;
    
    // ENHANCED: Larger bonus for CHs at scale (reduces hop count)
    // CHs have aggregated traffic handling, reducing total network load
    double chBonus = neighbor.isClusterHead ? 0.15 : 0.0;      // Was 0.1
    chBonus += neighbor.isSuperClusterHead ? 0.20 : 0.0;       // Was 0.1
    
    // Combined score with emphasis on forwarding via CHs
    double score = (DISTANCE_WEIGHT * progress) + 
                   (lmssWeight_ * lmss) + 
                   chBonus;
    
    return score;
}

uint32_t GeoRouter::GetPerimeterNextHop(uint32_t srcNode, uint32_t dstNode) {
    perimeterFallbackCount_++;
    
    // Simple perimeter routing: pick neighbor with best angle toward destination
    ns3::Ptr<ns3::MobilityModel> srcMm = fanetNodes.Get(srcNode)->GetObject<ns3::MobilityModel>();
    ns3::Ptr<ns3::MobilityModel> dstMm = fanetNodes.Get(dstNode)->GetObject<ns3::MobilityModel>();
    if (!srcMm || !dstMm) return UINT32_MAX;
    
    ns3::Vector srcPos = srcMm->GetPosition();
    ns3::Vector dstPos = dstMm->GetPosition();
    
    // Compute angle to destination
    double angleToDst = std::atan2(dstPos.y - srcPos.y, dstPos.x - srcPos.x);
    
    std::vector<GeoNeighbor> neighbors = GetNeighbors(srcNode);
    if (neighbors.empty()) return UINT32_MAX;
    
    // Sort by angle difference from destination angle (right-hand rule)
    double bestAngleDiff = 999;
    uint32_t bestNeighbor = UINT32_MAX;
    
    for (const auto& neighbor : neighbors) {
        double angleToNeighbor = std::atan2(
            neighbor.position.y - srcPos.y,
            neighbor.position.x - srcPos.x
        );
        
        // Right-hand rule: find first neighbor clockwise from destination
        double angleDiff = angleToNeighbor - angleToDst;
        if (angleDiff < 0) angleDiff += 2 * M_PI;
        
        if (angleDiff < bestAngleDiff && neighbor.lmssScore > 0.2) {
            bestAngleDiff = angleDiff;
            bestNeighbor = neighbor.nodeId;
        }
    }
    
    return bestNeighbor;
}

bool GeoRouter::IsInVoid(uint32_t srcNode, uint32_t dstNode) const {
    double srcToDst = GetDistance(fanetNodes.Get(srcNode), fanetNodes.Get(dstNode));
    
    std::vector<GeoNeighbor> neighbors = GetNeighbors(srcNode);
    for (const auto& neighbor : neighbors) {
        double neighborToDst = GetDistance(fanetNodes.Get(neighbor.nodeId), fanetNodes.Get(dstNode));
        if (neighborToDst < srcToDst) {
            return false;  // At least one neighbor makes progress
        }
    }
    return true;  // No neighbor is closer - we're in a void
}

uint32_t GeoRouter::GetHierarchicalNextHop(uint32_t srcNode, uint32_t dstNode) {
    hierarchicalRouteCount_++;
    
    // Check if in same cluster
    uint32_t srcCluster = nodeToCluster.count(srcNode) ? nodeToCluster[srcNode] : UINT32_MAX;
    uint32_t dstCluster = nodeToCluster.count(dstNode) ? nodeToCluster[dstNode] : UINT32_MAX;
    
    if (srcCluster == dstCluster && srcCluster != UINT32_MAX) {
        // Same cluster: use geo routing directly to destination
        return GetNextHop(srcNode, dstNode);
    }
    
    // Check if in same super-cluster
    if (IsInSameSuperCluster(srcNode, dstNode)) {
        // Same super-cluster: route to destination's cluster head
        uint32_t dstCH = GetClusterHead(dstNode);
        if (dstCH != srcNode) {
            return GetNextHop(srcNode, dstCH);
        }
        return GetNextHop(srcNode, dstNode);
    }
    
    // Different super-clusters: route to destination's super-cluster head
    uint32_t dstSCH = GetSuperClusterHead(dstNode);
    if (dstSCH != srcNode) {
        return GetNextHop(srcNode, dstSCH);
    }
    
    // Fallback to direct geo routing
    return GetNextHop(srcNode, dstNode);
}

void GeoRouter::PrintRoutingSummary() const {
    NS_LOG_UNCOND("\n--- Geographic Routing Summary ---");
    NS_LOG_UNCOND("Mode: " << (currentMode_ == GeoRoutingMode::GREEDY ? "Greedy" :
                                currentMode_ == GeoRoutingMode::LMSS_WEIGHTED ? "LMSS-Weighted" : "Perimeter"));
    NS_LOG_UNCOND("LMSS Weight: " << lmssWeight_);
    NS_LOG_UNCOND("Greedy successes: " << greedySuccessCount_);
    NS_LOG_UNCOND("Perimeter fallbacks: " << perimeterFallbackCount_);
    NS_LOG_UNCOND("Hierarchical routes: " << hierarchicalRouteCount_);
    
    double totalRoutes = greedySuccessCount_ + perimeterFallbackCount_;
    if (totalRoutes > 0) {
        NS_LOG_UNCOND("Greedy success rate: " 
                      << (100.0 * greedySuccessCount_ / totalRoutes) << "%");
    }
    NS_LOG_UNCOND("----------------------------------\n");
}

} // namespace myfanet
