/*
 * lmss.cc - 6-Component Enhanced LMSS Implementation
 * 
 * Improvements:
 * - Φ6 now uses variance-based velocity history for true predictability
 * - Spatial grid for O(n) neighbor discovery instead of O(n²)
 * - Link break tracking for stability scoring
 */

#include "lmss.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "cross_layer_controller.h"
#include "ns3/simulator.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace myfanet {

// === Helper: Compute velocity magnitude ===
static double VelocityMagnitude(const ns3::Vector& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

// === Component 1: Link Duration Score Φ1(i,j) ===
// Higher score = link has existed longer (more stable)
// NOW with link break penalty
double ComputeLinkDurationScore(uint32_t nodeI, uint32_t nodeJ)
{
    auto key = std::make_pair(std::min(nodeI, nodeJ), std::max(nodeI, nodeJ));
    double distance = GetDistance(fanetNodes.Get(nodeI), fanetNodes.Get(nodeJ));
    bool isConnected = distance < DEFAULT_COMM_RANGE;
    
    auto& history = linkHistories[key];
    
    // Initialize link history if new
    if (history.linkStartTime.IsZero()) {
        history.linkStartTime = ns3::Simulator::Now();
        history.lastDistance = distance;
        history.lastVelocity = GetNodeVelocity(fanetNodes.Get(nodeI));
        history.wasConnected = isConnected;
    }
    
    // Track link breaks
    if (history.wasConnected && !isConnected) {
        // Link just broke
        history.breakCount++;
        history.lastBreakTime = ns3::Simulator::Now();
    }
    history.wasConnected = isConnected;
    history.lastDistance = distance;
    
    // Link age in seconds
    double linkAge = (ns3::Simulator::Now() - history.linkStartTime).GetSeconds();
    // Normalize: cap at LINK_DURATION_WINDOW (10s)
    double baseScore = std::min(1.0, linkAge / LINK_DURATION_WINDOW);
    
    // Penalty for link breaks (each break reduces score by 10%, up to 50%)
    double breakPenalty = std::min(0.5, history.breakCount * 0.1);
    double phi1 = baseScore * (1.0 - breakPenalty);
    
    return std::max(0.0, phi1);
}

// === Component 2: Distance Stability Score Φ2(i,j) ===
// Higher score = nodes are closer (better link quality)
double ComputeDistanceStabilityScore(uint32_t nodeI, uint32_t nodeJ)
{
    double distance = GetDistance(fanetNodes.Get(nodeI), fanetNodes.Get(nodeJ));
    // Invert: closer = higher score
    double phi2 = 1.0 - std::min(1.0, distance / MAX_DISTANCE);
    return phi2;
}

// === Component 3: Relative Velocity Score Φ3(i,j) ===
// Higher score = nodes moving together (low relative velocity)
double ComputeRelativeVelocityScore(uint32_t nodeI, uint32_t nodeJ)
{
    ns3::Vector velI = GetNodeVelocity(fanetNodes.Get(nodeI));
    ns3::Vector velJ = GetNodeVelocity(fanetNodes.Get(nodeJ));
    
    // Relative velocity vector
    ns3::Vector relVel = velI - velJ;
    double deltaV = VelocityMagnitude(relVel);
    
    // Low relative velocity = high score
    double phi3 = 1.0 - std::min(1.0, deltaV / MAX_RELATIVE_VELOCITY);
    return phi3;
}

// === Component 4: Heading Alignment Score Φ4(i,j) ===
// Higher score = nodes flying in similar direction (cos(θ))
double ComputeHeadingAlignmentScore(uint32_t nodeI, uint32_t nodeJ)
{
    ns3::Vector velI = GetNodeVelocity(fanetNodes.Get(nodeI));
    ns3::Vector velJ = GetNodeVelocity(fanetNodes.Get(nodeJ));
    
    // Velocity magnitudes
    double lenI = VelocityMagnitude(velI);
    double lenJ = VelocityMagnitude(velJ);
    
    // If either stationary, moderate alignment (not penalized)
    if (lenI < 0.5 || lenJ < 0.5) return 0.5;
    
    // Dot product gives cosine of angle
    double dotProd = velI.x*velJ.x + velI.y*velJ.y + velI.z*velJ.z;
    double cosTheta = dotProd / (lenI * lenJ);
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    
    // Normalize [-1,1] to [0,1]: same heading = 1, opposite = 0
    double phi4 = (cosTheta + 1.0) / 2.0;
    return phi4;
}

// === Component 5: Altitude Stability Score Φ5(i,j) ===
// Higher score = nodes at similar altitude
double ComputeAltitudeStabilityScore(uint32_t nodeI, uint32_t nodeJ)
{
    ns3::Vector posI = GetNodePosition(fanetNodes.Get(nodeI));
    ns3::Vector posJ = GetNodePosition(fanetNodes.Get(nodeJ));
    
    // Altitude difference (z-coordinate)
    double altDiff = std::fabs(posI.z - posJ.z);
    // Similar altitude = high score
    double phi5 = 1.0 - std::min(1.0, altDiff / MAX_ALTITUDE_DIFF);
    return phi5;
}

// === Component 6: Flight-Path Predictability Score Φ6(i) ===
// IMPROVED: Now uses variance of velocity history
// Higher score = node follows predictable (constant velocity) path
double ComputeFlightPathPredictabilityScore(uint32_t nodeI)
{
    ns3::Vector currentVel = GetNodeVelocity(fanetNodes.Get(nodeI));
    auto& history = velocityHistories[nodeI];
    ns3::Time now = ns3::Simulator::Now();
    
    // Sample at configured interval
    if (history.lastSampleTime.IsZero() || 
        (now - history.lastSampleTime).GetSeconds() >= VELOCITY_SAMPLE_INTERVAL) {
        
        history.samples.push_back(currentVel);
        history.lastSampleTime = now;
        
        // Keep only recent samples
        if (history.samples.size() > VELOCITY_HISTORY_SIZE) {
            history.samples.erase(history.samples.begin());
        }
    }
    
    // Need minimum samples for meaningful variance
    if (history.samples.size() < 3) {
        // Not enough data - assume moderate predictability
        return 0.7;
    }
    
    // Compute mean velocity
    ns3::Vector mean(0, 0, 0);
    for (const auto& v : history.samples) {
        mean.x += v.x;
        mean.y += v.y;
        mean.z += v.z;
    }
    mean.x /= history.samples.size();
    mean.y /= history.samples.size();
    mean.z /= history.samples.size();
    
    // Compute variance of velocity (sum of squared deviations)
    double variance = 0.0;
    for (const auto& v : history.samples) {
        double dx = v.x - mean.x;
        double dy = v.y - mean.y;
        double dz = v.z - mean.z;
        variance += dx*dx + dy*dy + dz*dz;
    }
    variance /= history.samples.size();
    
    // Cache for potential reuse
    history.variance = variance;
    
    // Convert variance to score: low variance = high predictability
    // Use exponential decay: score = exp(-variance / scale)
    // Scale factor chosen so variance of 25 (5 m/s std dev) gives ~0.5 score
    const double VARIANCE_SCALE = 25.0;
    double phi6 = std::exp(-variance / VARIANCE_SCALE);
    
    return std::max(0.0, std::min(1.0, phi6));
}

// === Combined 6-Component Enhanced LMSS ===
// IMPROVED: Uses spatial grid for O(n) neighbor discovery
double ComputeEnhancedLMSS(uint32_t nodeI, double commRange)
{
    // Rebuild spatial grid if needed (first call or periodic)
    if (spatialGrid.NeedsRebuild()) {
        spatialGrid.Rebuild();
    }
    
    // Get candidate neighbors from spatial grid (O(1) average)
    std::vector<uint32_t> candidates = spatialGrid.GetCandidateNeighbors(nodeI, commRange);
    
    // Filter to actual neighbors within range
    std::vector<uint32_t> neighbors;
    for (uint32_t j : candidates) {
        if (GetDistance(fanetNodes.Get(nodeI), fanetNodes.Get(j)) < commRange) {
            neighbors.push_back(j);
        }
    }
    
    // No neighbors = isolated node
    if (neighbors.empty()) return 0.0;
    
    // Accumulate component scores across all neighbors
    double phi1_sum = 0, phi2_sum = 0, phi3_sum = 0, phi4_sum = 0, phi5_sum = 0;
    
    for (uint32_t nodeJ : neighbors) {
        phi1_sum += ComputeLinkDurationScore(nodeI, nodeJ);
        phi2_sum += ComputeDistanceStabilityScore(nodeI, nodeJ);
        phi3_sum += ComputeRelativeVelocityScore(nodeI, nodeJ);
        phi4_sum += ComputeHeadingAlignmentScore(nodeI, nodeJ);
        phi5_sum += ComputeAltitudeStabilityScore(nodeI, nodeJ);
    }
    
    // Average across neighbors
    double n = static_cast<double>(neighbors.size());
    double phi1_avg = phi1_sum / n;
    double phi2_avg = phi2_sum / n;
    double phi3_avg = phi3_sum / n;
    double phi4_avg = phi4_sum / n;
    double phi5_avg = phi5_sum / n;
    
    // Node-level predictability (not averaged)
    double phi6 = ComputeFlightPathPredictabilityScore(nodeI);
    
    // Get adaptive weights from Cross-Layer Controller
    // Weights adapt based on network conditions (jamming, mobility, etc.)
    const auto& clc = CrossLayerController::Instance();
    double beta1 = clc.GetAdaptiveBeta(0);  // Link duration
    double beta2 = clc.GetAdaptiveBeta(1);  // Distance stability
    double beta3 = clc.GetAdaptiveBeta(2);  // Relative velocity
    double beta4 = clc.GetAdaptiveBeta(3);  // Heading alignment
    double beta5 = clc.GetAdaptiveBeta(4);  // Altitude stability
    double beta6 = clc.GetAdaptiveBeta(5);  // Flight predictability
    
    // Final LMSS: weighted sum of all 6 components with ADAPTIVE weights
    double lmss = beta1 * phi1_avg 
                + beta2 * phi2_avg 
                + beta3 * phi3_avg 
                + beta4 * phi4_avg 
                + beta5 * phi5_avg 
                + beta6 * phi6;
    
    // Clamp to [0,1]
    return std::max(0.0, std::min(1.0, lmss));
}

// === Periodic Update Function ===
// Call this periodically to update spatial grid and cleanup old data
void UpdateSpatialGrid()
{
    spatialGrid.MarkNeedsRebuild();
    ns3::Simulator::Schedule(ns3::Seconds(1.0), &UpdateSpatialGrid);
}

// === Periodic Link History Cleanup ===
void CleanupLinkHistories()
{
    const double HISTORY_TIMEOUT = 30.0;
    
    auto it = linkHistories.begin();
    while (it != linkHistories.end()) {
        double age = (ns3::Simulator::Now() - it->second.linkStartTime).GetSeconds();
        if (age > HISTORY_TIMEOUT) {
            it = linkHistories.erase(it);
        } else {
            ++it;
        }
    }
    
    // Also cleanup old velocity histories
    auto vit = velocityHistories.begin();
    while (vit != velocityHistories.end()) {
        if (vit->second.samples.empty() || 
            (ns3::Simulator::Now() - vit->second.lastSampleTime).GetSeconds() > HISTORY_TIMEOUT) {
            vit = velocityHistories.erase(vit);
        } else {
            ++vit;
        }
    }
    
    ns3::Simulator::Schedule(ns3::Seconds(5.0), &CleanupLinkHistories);
}

} // namespace myfanet
