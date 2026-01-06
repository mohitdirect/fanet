/*
 * lmss.cc - 6-Component Enhanced LMSS Implementation
 */

#include "lmss.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "ns3/simulator.h"
#include <cmath>
#include <algorithm>

namespace myfanet {

// === Component 1: Link Duration Score Φ1(i,j) ===
// Higher score = link has existed longer (more stable)
double ComputeLinkDurationScore(uint32_t nodeI, uint32_t nodeJ)
{
    auto key = std::make_pair(std::min(nodeI, nodeJ), std::max(nodeI, nodeJ));
    
    // Initialize link history if new
    if (linkHistories.find(key) == linkHistories.end()) {
        linkHistories[key].linkStartTime = ns3::Simulator::Now();
        linkHistories[key].lastDistance = GetDistance(fanetNodes.Get(nodeI), fanetNodes.Get(nodeJ));
        linkHistories[key].lastVelocity = GetNodeVelocity(fanetNodes.Get(nodeI));
    }
    
    // Link age in seconds
    double linkAge = (ns3::Simulator::Now() - linkHistories[key].linkStartTime).GetSeconds();
    // Normalize: cap at LINK_DURATION_WINDOW (10s)
    double phi1 = std::min(1.0, linkAge / LINK_DURATION_WINDOW);
    
    return phi1;
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
    double deltaV = std::sqrt(relVel.x*relVel.x + relVel.y*relVel.y + relVel.z*relVel.z);
    
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
    double lenI = std::sqrt(velI.x*velI.x + velI.y*velI.y + velI.z*velI.z);
    double lenJ = std::sqrt(velJ.x*velJ.x + velJ.y*velJ.y + velJ.z*velJ.z);
    
    // If either stationary, no alignment benefit
    if (lenI < 0.01 || lenJ < 0.01) return 0.0;
    
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
// Higher score = node follows predictable (constant velocity) path
double ComputeFlightPathPredictabilityScore(uint32_t nodeI)
{
    ns3::Vector vel = GetNodeVelocity(fanetNodes.Get(nodeI));
    double speed = std::sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    
    // Stationary (speed < 0.5 m/s) = maximally predictable
    // Constant speed = predictable
    double phi6 = (speed < 0.5) ? 1.0 : 0.8;
    
    return phi6;
}

// === Combined 6-Component Enhanced LMSS ===
// LMSS(i) = average of 6 components across all neighbors
double ComputeEnhancedLMSS(uint32_t nodeI, double commRange)
{
    std::vector<uint32_t> neighbors;
    
    // Find all neighbors within communication range
    for (uint32_t j = 0; j < nNodes; ++j) {
        if (nodeI == j) continue;
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
    double phi1_avg = phi1_sum / neighbors.size();
    double phi2_avg = phi2_sum / neighbors.size();
    double phi3_avg = phi3_sum / neighbors.size();
    double phi4_avg = phi4_sum / neighbors.size();
    double phi5_avg = phi5_sum / neighbors.size();
    
    // Node-level predictability (not averaged)
    double phi6 = ComputeFlightPathPredictabilityScore(nodeI);
    
    // Final LMSS: weighted sum of all 6 components
    double lmss = BETA_1 * phi1_avg 
                + BETA_2 * phi2_avg 
                + BETA_3 * phi3_avg 
                + BETA_4 * phi4_avg 
                + BETA_5 * phi5_avg 
                + BETA_6 * phi6;
    
    // Clamp to [0,1]
    return std::max(0.0, std::min(1.0, lmss));
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
    
    ns3::Simulator::Schedule(ns3::Seconds(5.0), &CleanupLinkHistories);
}

} // namespace myfanet
