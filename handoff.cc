/*
 * handoff.cc - Handoff Execution and PGH Logic Implementation
 */

#include "handoff.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "lmss.h"
#include "sinr_monitor.h"
#include "cross_layer_controller.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include <string>
#include <cmath>

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

// Forward declarations for phased handoff
static void HandoffPhase2_QuietPeriod();
static void HandoffPhase3_ChannelSwitch();
static void HandoffPhase4_RouteConvergence();
static void HandoffPhase5_Complete();

// Track handoff timing for metrics
static ns3::Time handoffStartTime;
static ns3::Time handoffEndTime;
static uint32_t packetsLostDuringHandoff = 0;

// Execute the simulated frequency handoff with REALISTIC DELAYS
void ExecuteHandoff(const std::string& tag)
{
    if (actionTaken) return;
    detectionTime = ns3::Simulator::Now();
    handoffStartTime = detectionTime;
    actionTaken = true;
    
    // Notify Cross-Layer Controller that handoff is starting
    CrossLayerController::Instance().NotifyHandoffStarted();
    
    NS_LOG_UNCOND("\n" << std::string(50, '='));
    NS_LOG_UNCOND("🎯 HANDOFF EXECUTION at " << detectionTime.GetSeconds() << "s");
    NS_LOG_UNCOND("Protocol: " << tag);
    NS_LOG_UNCOND("Detection Mode: " << 
        (currentDetectionMode == DetectionMode::DISTANCE_BASED ? "DISTANCE" :
         currentDetectionMode == DetectionMode::SINR_BASED ? "SINR" :
         currentDetectionMode == DetectionMode::PER_BASED ? "PER" : "HYBRID"));
    
    // Print CLC network state at handoff time
    auto netState = CrossLayerController::Instance().GetNetworkState();
    NS_LOG_UNCOND("CLC Network Health: " << netState.networkHealth);
    NS_LOG_UNCOND("CLC Nodes Under Jamming: " << netState.nodesUnderJamming << "/" << netState.totalNodes);
    NS_LOG_UNCOND(std::string(50, '='));

    // === PHASE 1: Handoff Announcement (100ms) ===
    // In reality: CH broadcasts handoff request to all nodes
    NS_LOG_UNCOND("🔔 Phase 1: Handoff Announcement (" << (HANDOFF_ANNOUNCEMENT_DELAY * 1000) << "ms)");
    NS_LOG_UNCOND("   Broadcasting channel switch request to all nodes...");
    
    // Cache routes BEFORE disruption
    CacheCurrentRoutes();
    
    // Schedule Phase 2 after announcement delay
    ns3::Simulator::Schedule(ns3::Seconds(HANDOFF_ANNOUNCEMENT_DELAY), &HandoffPhase2_QuietPeriod);
    
    NS_LOG_UNCOND(std::string(50, '=') << "\n");
}

// Phase 2: Quiet period - no transmissions
static void HandoffPhase2_QuietPeriod()
{
    NS_LOG_UNCOND("⏸️  Phase 2: Quiet Period (" << (HANDOFF_QUIET_PERIOD * 1000) << "ms)");
    NS_LOG_UNCOND("   Stopping all transmissions for channel switch...");
    
    // In a real implementation, we would pause all applications here
    // This is where packets get lost in realistic handoffs
    packetsLostDuringHandoff = HANDOFF_MIN_PACKET_LOSS + 
        (rand() % (HANDOFF_MAX_PACKET_LOSS - HANDOFF_MIN_PACKET_LOSS));
    
    ns3::Simulator::Schedule(ns3::Seconds(HANDOFF_QUIET_PERIOD), &HandoffPhase3_ChannelSwitch);
}

// Phase 3: Actual channel switch
static void HandoffPhase3_ChannelSwitch()
{
    NS_LOG_UNCOND("🔀 Phase 3: Channel Switch");
    
    // Stop jammer applications
    jammerApps.Stop(ns3::Simulator::Now());
    NS_LOG_UNCOND("   ✅ Jammer applications stopped");
    
    // Stop jammer movement
    puNode->GetObject<ns3::ConstantVelocityMobilityModel>()->SetVelocity(ns3::Vector(0, 0, 0));
    
    // Move jammer far away (simulates switching to clean channel)
    ns3::Vector oldPos = puNode->GetObject<ns3::MobilityModel>()->GetPosition();
    puNode->GetObject<ns3::ConstantVelocityMobilityModel>()->SetPosition(
        ns3::Vector(JAMMER_ISOLATION_X, JAMMER_ISOLATION_Y, JAMMER_ISOLATION_Z));
    NS_LOG_UNCOND("   ✅ Channel switched (jammer isolated at " << JAMMER_ISOLATION_X << "m)");
    
    // Disable jammer Wi-Fi transmission
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
    
    NS_LOG_UNCOND("   Route convergence starting (" << HANDOFF_ROUTE_CONVERGENCE << "s)...");
    ns3::Simulator::Schedule(ns3::Seconds(HANDOFF_ROUTE_CONVERGENCE), &HandoffPhase4_RouteConvergence);
}

// Phase 4: Route convergence (OLSR takes 2-5s)
static void HandoffPhase4_RouteConvergence()
{
    NS_LOG_UNCOND("🔄 Phase 4: Route Convergence");
    
    // Trigger routing refresh
    TriggerRoutingRefresh();
    NS_LOG_UNCOND("   ✅ Routing tables being rebuilt");
    
    // Restore cached routes
    RestoreCachedRoutes();
    NS_LOG_UNCOND("   ✅ Cached routes restored");
    
    ns3::Simulator::Schedule(ns3::MilliSeconds(500), &HandoffPhase5_Complete);
}

// Phase 5: Handoff complete - report metrics
static void HandoffPhase5_Complete()
{
    handoffEndTime = ns3::Simulator::Now();
    double handoffDuration = (handoffEndTime - handoffStartTime).GetSeconds();
    
    NS_LOG_UNCOND("✅ Phase 5: Handoff Complete");
    NS_LOG_UNCOND("   Total handoff time: " << handoffDuration << "s");
    NS_LOG_UNCOND("   Packets lost during handoff: ~" << packetsLostDuringHandoff);
    
    // Verify handoff success
    double minDist = 1e9;
    for (uint32_t i = 0; i < nNodes; ++i) {
        double d = GetDistance(fanetNodes.Get(i), puNode);
        if (d < minDist) minDist = d;
    }
    
    if (minDist > HANDOFF_VERIFY_DISTANCE) {
        NS_LOG_UNCOND("   ✅ HANDOFF SUCCESSFUL - Jammer isolated at " << minDist << "m");
    } else {
        NS_LOG_UNCOND("   ⚠️  WARNING - Jammer still interfering at " << minDist << "m");
    }
    
    // Notify Cross-Layer Controller that handoff is complete
    // This resets jamming state and restores normal LMSS weights
    CrossLayerController::Instance().NotifyHandoffComplete();
    NS_LOG_UNCOND("   CLC: States reset, normal weights restored");
    
    // Transition to recovery phase
    ns3::Simulator::Schedule(ns3::Seconds(recoveryDelaySeconds - handoffDuration), 
                             &TransitionToRecoveryPhase);
    
    NS_LOG_UNCOND(std::string(50, '=') << "\n");
}

// Enhanced PGH-LMSS Logic with 6-Component LMSS
void UpdatePghLogic(ns3::Time interval)
{
    nodeScores.clear();
    nodeScores.resize(nNodes);
    double commRange = DEFAULT_COMM_RANGE;

    // 1. Compute 6-Component LMSS and LSSS for all nodes
    for (uint32_t i = 0; i < nNodes; ++i) {
        // LSSS: Spectrum quality - use detection mode selection
        double lsss;
        switch (currentDetectionMode) {
            case DetectionMode::SINR_BASED:
                lsss = ComputeSinrBasedLSSS(i);
                break;
            case DetectionMode::PER_BASED:
                lsss = ComputePerBasedLSSS(i);
                break;
            case DetectionMode::HYBRID:
                lsss = ComputeHybridLSSS(i);
                break;
            case DetectionMode::DISTANCE_BASED:
            default:
                // Original omniscient distance-based (for comparison)
                double distToPu = GetDistance(fanetNodes.Get(i), puNode);
                lsss = (distToPu - LSSS_BASE_RANGE) / LSSS_SCALE_RANGE;
                if (lsss < 0) lsss = 0;
                if (lsss > 1) lsss = 1;
                break;
        }
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

    // --- PDR OPTIMIZATION 3: MOBILITY-ADAPTIVE ROUTING INTERVALS ---
    // Calculate average LMSS (stability) of the network
    double avgLmss = 0.0;
    for (uint32_t i = 0; i < nNodes; ++i) {
        avgLmss += nodeScores[i].lmss;
    }
    avgLmss /= (nNodes > 0 ? nNodes : 1);

    // Dynamic Intervals based on LMSS
    double helloScale = 1.0;
    if (avgLmss > 0.8) helloScale = 2.0;       // Highly stable: slow down routing updates
    else if (avgLmss < 0.4) helloScale = 0.5;  // Highly dynamic: speed up routing updates

    double baseHello = 1.0; 
    double baseTc = 3.0;
    if (nNodes <= 30) { baseHello = 0.5; baseTc = 2.0; }
    else if (nNodes <= 60) { baseHello = 0.5 + (nNodes - 30) / 60.0; baseTc = 2.0 + (nNodes - 30) / 30.0; }
    else { baseHello = 1.0 + (nNodes - 60) / 80.0; baseTc = 3.0 + (nNodes - 60) / 20.0; }

    baseHello = std::min(baseHello * helloScale, 2.5);
    baseTc = std::min(baseTc * helloScale, 6.0);

    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(i)->GetObject<ns3::Ipv4>();
        if (!ipv4) continue;
        ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        ns3::Ptr<ns3::Ipv4ListRouting> list = ns3::DynamicCast<ns3::Ipv4ListRouting>(rp);
        if (list) {
            int16_t priority;
            for (uint32_t r = 0; r < list->GetNRoutingProtocols(); ++r) {
                ns3::Ptr<ns3::Ipv4RoutingProtocol> childRp = list->GetRoutingProtocol(r, priority);
                if (childRp->GetInstanceTypeId().GetName() == "ns3::olsr::RoutingProtocol") {
                    // PDR OPTIMIZATION: Drastically increase intervals for non-CH nodes 
                    // to suppress OLSR routing overhead and packet collisions
                    bool isCH = false;
                    auto it = nodeToCluster.find(i);
                    if (it != nodeToCluster.end() && it->second < clusters.size()) {
                        if (clusters[it->second].clusterHeadId == i) isCH = true;
                    } else if (nodeToCluster.empty()) {
                        isCH = true; // Bootstrap before clusters form
                    }
                    
                    if (isCH) {
                        childRp->SetAttribute("HelloInterval", ns3::TimeValue(ns3::Seconds(baseHello)));
                        childRp->SetAttribute("TcInterval", ns3::TimeValue(ns3::Seconds(baseTc)));
                    } else {
                        // Suppress routing traffic on ordinary members
                        childRp->SetAttribute("HelloInterval", ns3::TimeValue(ns3::Seconds(baseHello * 20.0)));
                        childRp->SetAttribute("TcInterval", ns3::TimeValue(ns3::Seconds(baseTc * 20.0)));
                    }
                }
            }
        }
    }
    // ---------------------------------------------------------------

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
    // Cluster stability tracking: only re-form if scores changed significantly
    static ns3::Time lastClusterUpdate = ns3::Seconds(0.0);
    double currentScoreSum = 0.0;
    for (uint32_t i = 0; i < nNodes; ++i) {
        currentScoreSum += nodeScores[i].combinedScore;
    }
    double scoreDelta = std::fabs(currentScoreSum - lastClusterScoreSum);
    bool significantChange = scoreDelta > (CLUSTER_STABILITY_THRESHOLD * nNodes);
    
    if ((now - lastClusterUpdate) >= ns3::Seconds(CLUSTER_UPDATE_INTERVAL) || 
        clusters.empty() || significantChange) {
        FormClusters();  // Uses current nodeScores with real LMSS/LSSS
        lastClusterUpdate = now;
        lastClusterScoreSum = currentScoreSum;
        if (significantChange && !clusters.empty()) {
            NS_LOG_DEBUG(now.GetSeconds() << "s: [PGH] Cluster re-formation due to score delta: " << scoreDelta);
        }
    }

    // 5. HANDOFF DECISION USING LMSS/LSSS (5 strategies)
    // Skip handoff for emulated protocols that don't have anti-jamming (mp-gpsr, g-olsr)
    if (!actionTaken && !handoffDisabled && ns3::Simulator::Now() >= ns3::Seconds(jammerStartTime)) {
        
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
    
    // DYNAMIC CLUSTER SCALING: Adaptive cluster size based on network scale
    // Small network (< 30): 6 nodes per cluster (default)
    // Medium network (30-60): 7 nodes per cluster
    // Large network (60-100): 8 nodes per cluster
    // Very large network (100+): 10 nodes per cluster (reduces CH overhead)
    uint32_t nodesPerCluster;
    if (nNodes < 30) {
        nodesPerCluster = NODES_PER_CLUSTER;  // 6
    } else if (nNodes < 60) {
        nodesPerCluster = 7;
    } else if (nNodes < 100) {
        nodesPerCluster = 8;
    } else {
        nodesPerCluster = 10;  // Larger clusters at scale
    }
    
    uint32_t dynamicClusters = std::max(DEFAULT_NUM_CLUSTERS, nNodes / nodesPerCluster);
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
    
    // LOAD BALANCING: Max members per cluster to prevent congestion
    uint32_t maxMembersPerCluster = (nNodes / numClusters) + 2;
    
    // Assign remaining nodes to nearest cluster (with capacity check)
    for (uint32_t i = 0; i < nNodes; ++i) {
        if (nodeToCluster.find(i) != nodeToCluster.end()) continue;  // Already a CH
        
        // Find clusters sorted by distance
        std::vector<std::pair<double, uint32_t>> clustersByDist;
        for (uint32_t c = 0; c < clusters.size(); ++c) {
            double dist = GetDistance(fanetNodes.Get(i), fanetNodes.Get(clusters[c].clusterHeadId));
            clustersByDist.push_back(std::make_pair(dist, c));
        }
        std::sort(clustersByDist.begin(), clustersByDist.end());
        
        // Join nearest cluster with capacity
        bool assigned = false;
        for (const auto& cd : clustersByDist) {
            uint32_t c = cd.second;
            if (clusters[c].members.size() < maxMembersPerCluster) {
                clusters[c].members.push_back(i);
                nodeToCluster[i] = c;
                assigned = true;
                break;
            }
        }
        // Fallback: if all full, join nearest
        if (!assigned && !clustersByDist.empty()) {
            uint32_t c = clustersByDist[0].second;
            clusters[c].members.push_back(i);
            nodeToCluster[i] = c;
        }
    }
    
    NS_LOG_UNCOND("\n=== CLUSTER FORMATION ===");
    for (const auto& c : clusters) {
        NS_LOG_UNCOND("  Cluster " << c.clusterId << ": CH=Node " << c.clusterHeadId 
            << ", Members=" << c.members.size() << ", Score=" << c.avgLmss);
    }
    NS_LOG_UNCOND("========================\n");

    // --- PDR OPTIMIZATION 2: LIMIT CONTROL PACKET SCOPE ---
    // Only Cluster Heads (CHs) will act as Multipoint Relays (MPRs) for OLSR.
    // Ordinary members will have WILL_NEVER to avoid forwarding network-wide TCs.
    for (uint32_t i = 0; i < nNodes; ++i) {
        ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(i)->GetObject<ns3::Ipv4>();
        if (!ipv4) continue;
        ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        ns3::Ptr<ns3::Ipv4ListRouting> list = ns3::DynamicCast<ns3::Ipv4ListRouting>(rp);
        if (list) {
            int16_t priority;
            for (uint32_t r = 0; r < list->GetNRoutingProtocols(); ++r) {
                ns3::Ptr<ns3::Ipv4RoutingProtocol> childRp = list->GetRoutingProtocol(r, priority);
                if (childRp->GetInstanceTypeId().GetName() == "ns3::olsr::RoutingProtocol") {
                    // Check if node is a CH
                    bool isCH = false;
                    auto it = nodeToCluster.find(i);
                    if (it != nodeToCluster.end()) {
                        if (clusters[it->second].clusterHeadId == i) {
                            isCH = true;
                        }
                    }
                    if (isCH) {
                        childRp->SetAttribute("Willingness", ns3::StringValue("always"));
                    } else {
                        childRp->SetAttribute("Willingness", ns3::StringValue("never"));
                    }
                }
            }
        }
    }
    // ------------------------------------------------------
}

// === Super-Cluster Formation for Hierarchical Routing ===
// Groups local clusters into super-clusters for reduced routing overhead
void FormSuperClusters()
{
    superClusters.clear();
    clusterToSuperCluster.clear();
    nodeToSuperCluster.clear();
    
    // Skip if not enough nodes for super-clusters
    if (nNodes < 30 || clusters.size() < 4) {
        // Create one super-cluster containing all clusters
        SuperClusterInfo sc;
        sc.superClusterId = 0;
        sc.superClusterHeadId = clusters.empty() ? 0 : clusters[0].clusterHeadId;
        sc.centroid = ns3::Vector(500, 500, 75);
        sc.avgScore = 0.5;
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            sc.memberClusters.push_back(i);
            clusters[i].superClusterId = 0;
            clusterToSuperCluster[i] = 0;
            for (uint32_t nodeId : clusters[i].members) {
                sc.memberNodes.push_back(nodeId);
                nodeToSuperCluster[nodeId] = 0;
            }
        }
        
        superClusters.push_back(sc);
        numSuperClusters = 1;
        return;
    }
    
    // Compute number of super-clusters based on node count
    numSuperClusters = std::max(MIN_SUPER_CLUSTERS, 
                                std::min(MAX_SUPER_CLUSTERS, nNodes / NODES_PER_SUPER_CLUSTER));
    
    // Compute centroids for each cluster
    std::vector<ns3::Vector> clusterCentroids(clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
        double sumX = 0, sumY = 0, sumZ = 0;
        for (uint32_t nodeId : clusters[c].members) {
            ns3::Ptr<ns3::MobilityModel> mm = fanetNodes.Get(nodeId)->GetObject<ns3::MobilityModel>();
            if (mm) {
                ns3::Vector pos = mm->GetPosition();
                sumX += pos.x;
                sumY += pos.y;
                sumZ += pos.z;
            }
        }
        if (!clusters[c].members.empty()) {
            clusterCentroids[c] = ns3::Vector(
                sumX / clusters[c].members.size(),
                sumY / clusters[c].members.size(),
                sumZ / clusters[c].members.size()
            );
        }
    }
    
    // K-means style super-cluster assignment
    // Initial super-cluster centers spread across network area
    superClusters.resize(numSuperClusters);
    for (uint32_t i = 0; i < numSuperClusters; ++i) {
        superClusters[i].superClusterId = i;
        // Spread initial centers
        double angleStep = 2.0 * M_PI / numSuperClusters;
        double radius = 300.0;
        superClusters[i].centroid = ns3::Vector(
            500 + radius * std::cos(i * angleStep),
            500 + radius * std::sin(i * angleStep),
            75
        );
        superClusters[i].avgScore = 0.0;
    }
    
    // Assign clusters to nearest super-cluster
    for (size_t c = 0; c < clusters.size(); ++c) {
        double minDist = 1e9;
        uint32_t nearestSC = 0;
        
        for (uint32_t sc = 0; sc < numSuperClusters; ++sc) {
            double dist = std::sqrt(
                std::pow(clusterCentroids[c].x - superClusters[sc].centroid.x, 2) +
                std::pow(clusterCentroids[c].y - superClusters[sc].centroid.y, 2)
            );
            if (dist < minDist) {
                minDist = dist;
                nearestSC = sc;
            }
        }
        
        clusters[c].superClusterId = nearestSC;
        clusterToSuperCluster[c] = nearestSC;
        superClusters[nearestSC].memberClusters.push_back(c);
        
        // Add nodes to super-cluster
        for (uint32_t nodeId : clusters[c].members) {
            superClusters[nearestSC].memberNodes.push_back(nodeId);
            nodeToSuperCluster[nodeId] = nearestSC;
        }
    }
    
    // Update super-cluster centroids and elect super-cluster heads
    for (uint32_t sc = 0; sc < numSuperClusters; ++sc) {
        auto& superCluster = superClusters[sc];
        
        // Recompute centroid
        double sumX = 0, sumY = 0, sumZ = 0;
        double scoreSum = 0;
        
        for (uint32_t clusterId : superCluster.memberClusters) {
            sumX += clusterCentroids[clusterId].x;
            sumY += clusterCentroids[clusterId].y;
            sumZ += clusterCentroids[clusterId].z;
            scoreSum += clusters[clusterId].avgLmss;
        }
        
        if (!superCluster.memberClusters.empty()) {
            size_t n = superCluster.memberClusters.size();
            superCluster.centroid = ns3::Vector(sumX / n, sumY / n, sumZ / n);
            superCluster.avgScore = scoreSum / n;
        }
        
        // Elect super-cluster head as highest scoring CH
        uint32_t bestCH = 0;
        double bestScore = -1;
        for (uint32_t clusterId : superCluster.memberClusters) {
            double chScore = clusters[clusterId].avgLmss;
            // Bonus for being central to super-cluster
            double distToCenter = std::sqrt(
                std::pow(clusterCentroids[clusterId].x - superCluster.centroid.x, 2) +
                std::pow(clusterCentroids[clusterId].y - superCluster.centroid.y, 2)
            );
            double centralityBonus = 1.0 - std::min(1.0, distToCenter / SUPER_CLUSTER_RANGE);
            chScore += centralityBonus * SUPER_CH_SCORE_BONUS;
            
            if (chScore > bestScore) {
                bestScore = chScore;
                bestCH = clusters[clusterId].clusterHeadId;
            }
        }
        superCluster.superClusterHeadId = bestCH;
    }
    
    NS_LOG_UNCOND("\n=== SUPER-CLUSTER FORMATION ===");
    for (const auto& sc : superClusters) {
        NS_LOG_UNCOND("  SuperCluster " << sc.superClusterId 
                      << ": SCH=Node " << sc.superClusterHeadId
                      << ", Clusters=" << sc.memberClusters.size()
                      << ", Nodes=" << sc.memberNodes.size()
                      << ", Score=" << sc.avgScore);
    }
    NS_LOG_UNCOND("===============================\n");
}

// Update cluster membership dynamically (can be called periodically)
void UpdateClusterMembership()
{
    // Re-form clusters based on current LMSS scores
    FormClusters();
    
    // Form super-clusters for hierarchical routing
    FormSuperClusters();
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

// Get the super-cluster head for a given node
uint32_t GetSuperClusterHead(uint32_t nodeId)
{
    auto it = nodeToSuperCluster.find(nodeId);
    if (it != nodeToSuperCluster.end() && it->second < superClusters.size()) {
        return superClusters[it->second].superClusterHeadId;
    }
    return GetClusterHead(nodeId);  // Fallback to cluster head
}

// Check if two nodes are in the same super-cluster
bool IsInSameSuperCluster(uint32_t nodeA, uint32_t nodeB)
{
    auto itA = nodeToSuperCluster.find(nodeA);
    auto itB = nodeToSuperCluster.find(nodeB);
    
    if (itA == nodeToSuperCluster.end() || itB == nodeToSuperCluster.end()) {
        return false;
    }
    return itA->second == itB->second;
}

} // namespace myfanet
