/*
 * lmss.h - 6-Component Enhanced LMSS Calculations
 * 
 * LMSS = β1*Φ1 + β2*Φ2 + β3*Φ3 + β4*Φ4 + β5*Φ5 + β6*Φ6
 * where:
 *   Φ1: Link Duration
 *   Φ2: Distance Stability
 *   Φ3: Relative Velocity
 *   Φ4: Heading Alignment
 *   Φ5: Altitude Stability
 *   Φ6: Flight-Path Predictability
 */

#ifndef MYFANET_LMSS_H
#define MYFANET_LMSS_H

#include <cstdint>

namespace myfanet {

// === Component 1: Link Duration Score Φ1(i,j) ===
double ComputeLinkDurationScore(uint32_t nodeI, uint32_t nodeJ);

// === Component 2: Distance Stability Score Φ2(i,j) ===
double ComputeDistanceStabilityScore(uint32_t nodeI, uint32_t nodeJ);

// === Component 3: Relative Velocity Score Φ3(i,j) ===
double ComputeRelativeVelocityScore(uint32_t nodeI, uint32_t nodeJ);

// === Component 4: Heading Alignment Score Φ4(i,j) ===
double ComputeHeadingAlignmentScore(uint32_t nodeI, uint32_t nodeJ);

// === Component 5: Altitude Stability Score Φ5(i,j) ===
double ComputeAltitudeStabilityScore(uint32_t nodeI, uint32_t nodeJ);

// === Component 6: Flight-Path Predictability Score Φ6(i) ===
double ComputeFlightPathPredictabilityScore(uint32_t nodeI);

// === Combined 6-Component Enhanced LMSS ===
double ComputeEnhancedLMSS(uint32_t nodeI, double commRange);

// === Periodic Link History Cleanup ===
void CleanupLinkHistories();

} // namespace myfanet

#endif // MYFANET_LMSS_H
