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
void CrEdLoop()
{
    if (actionTaken) return;
    if (ns3::Simulator::Now() < ns3::Seconds(jammerStartTime)) {
        ns3::Simulator::Schedule(ns3::Seconds(0.1), &CrEdLoop);
        return;
    }
    double minDist = 1e9;
    for (uint32_t i = 0; i < nNodes; ++i) {
        minDist = std::min(minDist, GetDistance(fanetNodes.Get(i), puNode));
    }
    if (minDist < crEdDetectRange) {
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-ED] Reactive detection at " << minDist << " m");
        ExecuteHandoff("CR-ED");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(0.1), &CrEdLoop);
}

// CR-SCAN: Periodic scanning loop
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
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-SCAN] Scan trigger at " << minDist << " m");
        ExecuteHandoff("CR-SCAN");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(crScanInterval), &CrScanLoop);
}

// CR-COOP: Cooperative sensing loop (quorum)
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
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [CR-COOP] Quorum reached (" << affected << "/" << nNodes << " nodes affected)");
        ExecuteHandoff("CR-COOP");
        return;
    }
    ns3::Simulator::Schedule(ns3::Seconds(0.2), &CrCoopLoop);
}

} // namespace myfanet
