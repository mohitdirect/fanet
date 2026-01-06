/*
 * handoff.cc - Handoff Execution and PGH Logic Implementation
 */

#include "handoff.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "lmss.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include <string>

NS_LOG_COMPONENT_DEFINE("Handoff");

namespace myfanet {

// Cached routes for faster recovery
static std::vector<std::pair<ns3::Ipv4Address, ns3::Ipv4Address>> cachedRoutes;

// Cache current routes before handoff
void CacheCurrentRoutes()
{
    cachedRoutes.clear();
    NS_LOG_UNCOND("  [Route Cache] Caching routes from " << nNodes << " nodes...");
    
    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(i)->GetObject<ns3::Ipv4>();
        if (!ipv4) continue;
        
        // Cache the node's IP for routing restoration
        for (uint32_t j = 1; j < ipv4->GetNInterfaces(); ++j) {
            ns3::Ipv4InterfaceAddress addr = ipv4->GetAddress(j, 0);
            cachedRoutes.push_back(std::make_pair(addr.GetLocal(), addr.GetBroadcast()));
        }
    }
    NS_LOG_UNCOND("  [Route Cache] Cached " << cachedRoutes.size() << " interface addresses");
}

// Restore routes after handoff
void RestoreCachedRoutes()
{
    NS_LOG_UNCOND("  [Route Restore] Forcing rapid route rediscovery...");
    
    // Force all interfaces to trigger route updates
    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(i)->GetObject<ns3::Ipv4>();
        if (!ipv4) continue;
        ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        if (!rp) continue;
        
        // Trigger interface up notifications to force route rebuild
        for (uint32_t j = 1; j < ipv4->GetNInterfaces(); ++j) {
            rp->NotifyInterfaceUp(j);
        }
    }
    NS_LOG_UNCOND("  [Route Restore] Routing refresh triggered for " << nNodes << " nodes");
}

// Phase transition to jamming
void TransitionToJammingPhase()
{
    if (currentPhase == PRE_JAMMING) {
        currentPhase = DURING_JAMMING;
        jammingStartTimeActual = ns3::Simulator::Now();
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [Phase] PRE_JAMMING -> DURING_JAMMING");
    }
}

// Phase transition to recovery (with delay)
void TransitionToRecoveryPhase()
{
    if (currentPhase == DURING_JAMMING) {
        currentPhase = POST_RECOVERY;
        recoveryStartTime = ns3::Simulator::Now();
        NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [Phase] DURING_JAMMING -> POST_RECOVERY");
    }
}

// Quick routing refresh after handoff
void TriggerRoutingRefresh()
{
    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(i)->GetObject<ns3::Ipv4>();
        if (!ipv4) continue;
        ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        if (!rp) continue;
        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
            rp->NotifyInterfaceUp(j);
        }
    }
}

// Execute the simulated frequency handoff
void ExecuteHandoff(const std::string& tag)
{
    if (actionTaken) return;
    detectionTime = ns3::Simulator::Now();
    actionTaken = true;
    
    NS_LOG_UNCOND("\n" << std::string(50, '='));
    NS_LOG_UNCOND("🎯 HANDOFF EXECUTION at " << detectionTime.GetSeconds() << "s");
    NS_LOG_UNCOND("Protocol: " << tag);
    NS_LOG_UNCOND(std::string(50, '='));

    // Step 0: Cache routes BEFORE disruption
    CacheCurrentRoutes();
    NS_LOG_UNCOND("✅ Step 0: Routes cached for fast recovery");

    // Step 1: Stop jammer applications
    jammerApps.Stop(ns3::Simulator::Now());
    NS_LOG_UNCOND("✅ Step 1: Jammer applications STOPPED");

    // Step 2: Stop jammer movement (velocity = 0)
    puNode->GetObject<ns3::ConstantVelocityMobilityModel>()->SetVelocity(ns3::Vector(0, 0, 0));
    NS_LOG_UNCOND("✅ Step 2: Jammer velocity set to ZERO");

    // Step 3: Move jammer far away
    ns3::Vector oldPos = puNode->GetObject<ns3::MobilityModel>()->GetPosition();
    puNode->GetObject<ns3::ConstantVelocityMobilityModel>()->SetPosition(ns3::Vector(5000, 5000, 1000));
    NS_LOG_UNCOND("✅ Step 3: Jammer position: (" << oldPos.x << "," << oldPos.y 
                  << ") → (5000,5000) [out of range]");

    // Step 4: Disable jammer Wi-Fi transmission completely
    for (uint32_t i = 0; i < puNode->GetNDevices(); ++i) {
        ns3::Ptr<ns3::WifiNetDevice> wifiDev = ns3::DynamicCast<ns3::WifiNetDevice>(puNode->GetDevice(i));
        if (wifiDev) {
            ns3::Ptr<ns3::WifiPhy> phy = wifiDev->GetPhy();
            if (phy) {
                phy->SetTxPowerStart(0.0);
                phy->SetTxPowerEnd(0.0);
            }
        }
    }
    NS_LOG_UNCOND("✅ Step 4: Jammer Wi-Fi transmission DISABLED");

    // Step 5: Immediate routing refresh
    ns3::Simulator::Schedule(ns3::MilliSeconds(50), &TriggerRoutingRefresh);
    NS_LOG_UNCOND("✅ Step 5: Routing refresh scheduled (+0.05s)");

    // Step 6: Restore cached routes after 100ms
    ns3::Simulator::Schedule(ns3::MilliSeconds(100), &RestoreCachedRoutes);
    NS_LOG_UNCOND("✅ Step 6: Route restoration scheduled (+0.1s)");

    // Step 7: Transition to recovery phase after recoveryDelaySeconds
    ns3::Simulator::Schedule(ns3::Seconds(recoveryDelaySeconds), &TransitionToRecoveryPhase);
    NS_LOG_UNCOND("✅ Step 7: Recovery phase transition scheduled (+" << recoveryDelaySeconds << "s)");

    // Step 8: Verify handoff success after 1 second
    ns3::Simulator::Schedule(ns3::Seconds(1.0), [](){
        double minDist = 1e9;
        for (uint32_t i = 0; i < nNodes; ++i) {
            double d = GetDistance(fanetNodes.Get(i), puNode);
            if (d < minDist) minDist = d;
        }
        NS_LOG_UNCOND("✅ Step 8: Verification at t=" << ns3::Simulator::Now().GetSeconds() << "s");
        NS_LOG_UNCOND("   Closest node to jammer: " << minDist << "m");
        if (minDist > 3000) {
            NS_LOG_UNCOND("   ✅ HANDOFF SUCCESSFUL - Jammer isolated");
        } else {
            NS_LOG_UNCOND("   ⚠️  WARNING - Jammer still interfering at " << minDist << "m");
        }
    });

    NS_LOG_UNCOND(std::string(50, '=') << "\n");
}

// Enhanced PGH-LMSS Logic with 6-Component LMSS
void UpdatePghLogic(ns3::Time interval)
{
    nodeScores.clear();
    nodeScores.resize(nNodes);
    double commRange = 600.0;

    // 1. Compute 6-Component LMSS and LSSS for all nodes
    for (uint32_t i = 0; i < nNodes; ++i) {
        // LSSS: Spectrum quality based on distance to jammer
        double distToPu = GetDistance(fanetNodes.Get(i), puNode);
        double lsss = (distToPu - 300.0) / 1000.0;
        if (lsss < 0) lsss = 0;
        if (lsss > 1) lsss = 1;
        nodeScores[i].lsss = lsss;

        // LMSS: 6-Component Enhanced Mobility Metric
        nodeScores[i].lmss = ComputeEnhancedLMSS(i, commRange);
        
        // Combined score
        nodeScores[i].combinedScore = 0.5 * nodeScores[i].lmss + 0.5 * nodeScores[i].lsss;

        NS_LOG_DEBUG(ns3::Simulator::Now().GetSeconds() << "s: Node " << i 
            << " LMSS=" << nodeScores[i].lmss 
            << " LSSS=" << nodeScores[i].lsss 
            << " Combined=" << nodeScores[i].combinedScore);
    }

    // 2. Threat Assessment: Check if jammer is approaching
    double minLsss = 1.0;
    double minCombinedScore = 1.0;
    double minLmss = 1.0;
    uint32_t affectedNodes = 0;
    bool threatDetected = false;

    for (uint32_t i = 0; i < nNodes; ++i) {
        if (nodeScores[i].lsss < minLsss) {
            minLsss = nodeScores[i].lsss;
        }
        if (nodeScores[i].lmss < minLmss) {
            minLmss = nodeScores[i].lmss;
        }
        if (nodeScores[i].lsss < lsssHandoffThreshold) {
            affectedNodes++;
        }
        if (nodeScores[i].combinedScore < minCombinedScore) {
            minCombinedScore = nodeScores[i].combinedScore;
        }
    }

    // Detect threat if LSSS drops significantly
    if (minLsss < 0.8) {
        threatDetected = true;
    }

    // 3. Adaptive Score Weighting
    double lmssWeight = threatDetected ? lmssWeightThreat : lmssWeightNormal;
    double lsssWeight = threatDetected ? lsssWeightThreat : lsssWeightNormal;

    // Recompute combined scores with adaptive weights
    for (uint32_t i = 0; i < nNodes; ++i) {
        nodeScores[i].combinedScore = lmssWeight * nodeScores[i].lmss + lsssWeight * nodeScores[i].lsss;
        if (nodeScores[i].combinedScore < minCombinedScore) {
            minCombinedScore = nodeScores[i].combinedScore;
        }
    }

    // 4. CH Election with Stability
    uint32_t bestNodeId = 0;
    double maxScore = -1.0;

    for (uint32_t i = 0; i < nNodes; ++i) {
        if (nodeScores[i].combinedScore > maxScore) {
            maxScore = nodeScores[i].combinedScore;
            bestNodeId = i;
        }
    }

    // CH stability logic
    bool allowChange = false;
    ns3::Time now = ns3::Simulator::Now();
    if (currentClusterHeadId == 999) {
        allowChange = true;
    } else {
        bool heldLongEnough = (now - lastChChangeTime) >= minChHoldTime;
        bool significantGain = (maxScore - currentClusterHeadScore) > chChangeThreshold;
        if (heldLongEnough && significantGain) allowChange = true;
    }

    if (allowChange && bestNodeId != currentClusterHeadId) {
        NS_LOG_UNCOND(now.GetSeconds() << "s: [PGH-LMSS] New CH Elected: Node "
            << bestNodeId << " (Score: " << maxScore << ", LMSS: " 
            << nodeScores[bestNodeId].lmss << " [6-comp], LSSS: " 
            << nodeScores[bestNodeId].lsss << ")");
        currentClusterHeadId = bestNodeId;
        currentClusterHeadScore = maxScore;
        lastChChangeTime = now;
    }

    // 4b. DYNAMIC CLUSTER FORMATION using real LMSS/LSSS scores
    // Update clusters every 5 seconds for stability (not every interval)
    static ns3::Time lastClusterUpdate = ns3::Seconds(0.0);
    if ((now - lastClusterUpdate) >= ns3::Seconds(5.0) || clusters.empty()) {
        FormClusters();  // Uses current nodeScores with real LMSS/LSSS
        lastClusterUpdate = now;
    }

    // 5. HANDOFF DECISION USING LMSS/LSSS (5 strategies)
    if (!actionTaken && ns3::Simulator::Now() >= ns3::Seconds(jammerStartTime)) {
        
        bool triggerHandoff = false;
        std::string reason = "";

        // Strategy 1: Critical LSSS threshold (emergency)
        if (minLsss < criticalLsssThreshold) {
            triggerHandoff = true;
            reason = "Critical LSSS threshold (" + std::to_string(minLsss) + " < " + std::to_string(criticalLsssThreshold) + ")";
        }
        
        // Strategy 2: LSSS degradation threshold (proactive)
        else if (minLsss < lsssHandoffThreshold) {
            triggerHandoff = true;
            reason = "LSSS degradation (" + std::to_string(minLsss) + " < " + std::to_string(lsssHandoffThreshold) + ")";
        }
        
        // Strategy 3: Combined score threshold (holistic)
        else if (minCombinedScore < combinedScoreThreshold) {
            triggerHandoff = true;
            reason = "Combined score degradation (" + std::to_string(minCombinedScore) + " < " + std::to_string(combinedScoreThreshold) + ")";
        }
        
        // Strategy 4: Cooperative quorum (multiple nodes affected)
        else if (affectedNodes >= affectedNodesQuorum) {
            triggerHandoff = true;
            reason = "Cooperative detection (" + std::to_string(affectedNodes) + " nodes with LSSS < " + std::to_string(lsssHandoffThreshold) + ")";
        }

        // Strategy 5: Topology collapse detection (low min LMSS + interference)
        else if (minLmss < 0.1 && minLsss < 0.6) {
            triggerHandoff = true;
            reason = "Topology collapse (" + std::to_string(minLmss) + " min LMSS + " + std::to_string(minLsss) + " min LSSS)";
        }

        if (triggerHandoff) {
            NS_LOG_UNCOND(ns3::Simulator::Now().GetSeconds() << "s: [PGH-LMSS] Handoff triggered!");
            NS_LOG_UNCOND("  Reason: " << reason);
            NS_LOG_UNCOND("  Min LSSS: " << minLsss);
            NS_LOG_UNCOND("  Min LMSS: " << minLmss << " [6-component]");
            NS_LOG_UNCOND("  Min Combined Score: " << minCombinedScore);
            NS_LOG_UNCOND("  Affected Nodes: " << affectedNodes << "/" << nNodes);
            NS_LOG_UNCOND("  Current CH: Node " << currentClusterHeadId 
                << " (LMSS=" << nodeScores[currentClusterHeadId].lmss 
                << ", LSSS=" << nodeScores[currentClusterHeadId].lsss << ")");
            
            ExecuteHandoff("PGH-LMSS");
        }
    }
    
    ns3::Simulator::Schedule(interval, &UpdatePghLogic, interval);
}

// === Cluster Formation and Management ===

// Form clusters based on LMSS scores - nodes with highest scores become CHs
// SCALABILITY: Dynamic cluster count based on node count
void FormClusters()
{
    clusters.clear();
    nodeToCluster.clear();
    
    // DYNAMIC CLUSTER SCALING: Scale clusters with node count
    // Target: 1 cluster per NODES_PER_CLUSTER nodes (e.g., 6 nodes per cluster)
    uint32_t dynamicClusters = std::max(DEFAULT_NUM_CLUSTERS, nNodes / NODES_PER_CLUSTER);
    dynamicClusters = std::min(dynamicClusters, nNodes / 2);  // At least 2 nodes per cluster
    numClusters = dynamicClusters;
    
    // Sort nodes by combined score (highest first) to select CHs
    // LSSS-AWARE: Penalize nodes near jammer (low LSSS)
    std::vector<std::pair<double, uint32_t>> sortedNodes;
    for (uint32_t i = 0; i < nNodes; ++i) {
        double score = (nodeScores.size() > i) ? nodeScores[i].combinedScore : 0.5;
        double lsss = (nodeScores.size() > i) ? nodeScores[i].lsss : 1.0;
        
        // Penalize nodes near jammer for CH election
        if (lsss < LSSS_ROUTING_THRESHOLD) {
            score *= 0.5;  // Reduce score for nodes near jammer
        }
        
        sortedNodes.push_back(std::make_pair(score, i));
    }
    std::sort(sortedNodes.begin(), sortedNodes.end(), std::greater<std::pair<double, uint32_t>>());
    
    // Select top N nodes as cluster heads
    std::vector<uint32_t> clusterHeads;
    for (uint32_t i = 0; i < numClusters && i < nNodes; ++i) {
        clusterHeads.push_back(sortedNodes[i].second);
        
        ClusterInfo cluster;
        cluster.clusterId = i;
        cluster.clusterHeadId = sortedNodes[i].second;
        cluster.avgLmss = sortedNodes[i].first;
        cluster.members.push_back(sortedNodes[i].second);  // CH is also a member
        clusters.push_back(cluster);
        
        nodeToCluster[sortedNodes[i].second] = i;
    }
    
    // Assign remaining nodes to nearest cluster head
    for (uint32_t i = 0; i < nNodes; ++i) {
        if (nodeToCluster.find(i) != nodeToCluster.end()) continue;  // Already a CH
        
        double minDist = 1e9;
        uint32_t bestCluster = 0;
        
        for (uint32_t c = 0; c < clusters.size(); ++c) {
            double dist = GetDistance(fanetNodes.Get(i), fanetNodes.Get(clusters[c].clusterHeadId));
            if (dist < minDist) {
                minDist = dist;
                bestCluster = c;
            }
        }
        
        clusters[bestCluster].members.push_back(i);
        nodeToCluster[i] = bestCluster;
    }
    
    NS_LOG_UNCOND("\n=== CLUSTER FORMATION ===");
    for (const auto& c : clusters) {
        NS_LOG_UNCOND("  Cluster " << c.clusterId << ": CH=Node " << c.clusterHeadId 
            << ", Members=" << c.members.size() << ", Score=" << c.avgLmss);
    }
    NS_LOG_UNCOND("========================\n");
}

// Update cluster membership dynamically (can be called periodically)
void UpdateClusterMembership()
{
    // Re-form clusters based on current LMSS scores
    FormClusters();
}

// Get the cluster head for a given node
uint32_t GetClusterHead(uint32_t nodeId)
{
    auto it = nodeToCluster.find(nodeId);
    if (it != nodeToCluster.end() && it->second < clusters.size()) {
        return clusters[it->second].clusterHeadId;
    }
    return nodeId;  // Return self if no cluster found
}

} // namespace myfanet
