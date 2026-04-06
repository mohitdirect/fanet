/*
 * cr_protocols.cc - Cognitive Radio Protocol Implementations
 */

#include "cr_protocols.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "handoff.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE("CrProtocols");

namespace myfanet {

// CR-ED: Reactive energy-detection loop
// IMPROVED: Now includes phase transition like PGH
void CrEdLoop()
{
    if (actionTaken) return;
    if (ns3::Simulator::Now() < ns3::Seconds(jammerStartTime)) {
        ns3::Simulator::Schedule(ns3::Seconds(0.1), &CrEdLoop);
        return;
    }
    
    // Check if any node detects jammer within range
    double minDist = 1e9;
    uint32_t closestNode = 0;
    for (uint32_t i = 0; i < nNodes; ++i) {
        double dist = GetDistance(fanetNodes.Get(i), puNode);
        if (dist < minDist) {
            minDist = dist;
            closestNode = i;
        }
    }
    
    if (minDist < crEdDetectRange) {
        // Transition to jamming phase BEFORE handoff (like PGH)
        if (currentPhase != DURING_JAMMING) {
            TransitionToJammingPhase();
        }
        
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-ED] Reactive detection");
        NS_LOG_UNCOND("   Closest node: " << closestNode << " at " << minDist << "m from jammer");
        NS_LOG_UNCOND("   Detection threshold: " << crEdDetectRange << "m");
        
        // Execute handoff with realistic delays (same as PGH)
        ExecuteHandoff("CR-ED");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(0.1), &CrEdLoop);
}

// CR-SCAN: Periodic scanning loop
// IMPROVED: Now includes phase transition like PGH
void CrScanLoop()
{
    if (actionTaken) return;
    if (ns3::Simulator::Now() < ns3::Seconds(jammerStartTime)) {
        ns3::Simulator::Schedule(ns3::Seconds(crScanInterval), &CrScanLoop);
        return;
    }
    double minDist = 1e9;
    for (uint32_t i = 0; i < nNodes; ++i) {
        minDist = std::min(minDist, GetDistance(fanetNodes.Get(i), puNode));
    }
    if (minDist < crScanRange) {
        // Transition to jamming phase BEFORE handoff (like PGH)
        if (currentPhase != DURING_JAMMING) {
            TransitionToJammingPhase();
        }
        
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-SCAN] Scan trigger at " << minDist << " m");
        ExecuteHandoff("CR-SCAN");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(crScanInterval), &CrScanLoop);
}

// CR-COOP: Cooperative sensing loop (quorum)
// IMPROVED: Now includes phase transition like PGH
void CrCoopLoop()
{
    if (actionTaken) return;
    if (ns3::Simulator::Now() < ns3::Seconds(jammerStartTime)) {
        ns3::Simulator::Schedule(ns3::Seconds(0.2), &CrCoopLoop);
        return;
    }
    uint32_t affected = 0;
    for (uint32_t i = 0; i < nNodes; ++i) {
        if (GetDistance(fanetNodes.Get(i), puNode) < crCoopRange) {
            affected++;
        }
    }
    uint32_t quorum = std::max<uint32_t>(crCoopQuorum, std::max<uint32_t>(1, nNodes / 10));
    if (affected >= quorum) {
        // Transition to jamming phase BEFORE handoff (like PGH)
        if (currentPhase != DURING_JAMMING) {
            TransitionToJammingPhase();
        }
        
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-COOP] Quorum reached (" << affected << "/" << nNodes << " nodes affected)");
        ExecuteHandoff("CR-COOP");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(0.2), &CrCoopLoop);
}

} // namespace myfanet
