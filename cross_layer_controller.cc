/*
 * cross_layer_controller.cc - Cross-Layer Controller Implementation
 * 
 * Implements centralized cross-layer information sharing and control
 * for improved FANET performance under jamming conditions.
 */

#include "cross_layer_controller.h"
#include "config.h"
#include "types.h"
#include "lmss.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-mac.h"
#include "ns3/internet-module.h"
#include <algorithm>
#include <numeric>
#include <iomanip>

NS_LOG_COMPONENT_DEFINE("CrossLayerController");

namespace myfanet {

// === Singleton Instance ===
CrossLayerController* CrossLayerController::instance_ = nullptr;

CrossLayerController& CrossLayerController::Instance() {
    if (!instance_) {
        instance_ = new CrossLayerController();
    }
    return *instance_;
}

// === Initialization ===
void CrossLayerController::Initialize(uint32_t numNodes) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    numNodes_ = numNodes;
    nodeStates_.clear();
    
    // Initialize state for each node
    for (uint32_t i = 0; i < numNodes; ++i) {
        CrossLayerState state;
        state.lastPhyUpdate = ns3::Simulator::Now();
        state.lastMacUpdate = ns3::Simulator::Now();
        state.lastNetUpdate = ns3::Simulator::Now();
        state.lastAppUpdate = ns3::Simulator::Now();
        nodeStates_[i] = state;
    }
    
    // Initialize LMSS weights
    normalWeights_.beta[0] = 0.15;  // Link duration
    normalWeights_.beta[1] = 0.20;  // Distance stability
    normalWeights_.beta[2] = 0.15;  // Relative velocity
    normalWeights_.beta[3] = 0.15;  // Heading alignment
    normalWeights_.beta[4] = 0.15;  // Altitude stability
    normalWeights_.beta[5] = 0.20;  // Flight predictability
    
    // Jamming-optimized weights prioritize distance and altitude (3D escape)
    jammingWeights_.beta[0] = 0.10;  // Link duration (less important)
    jammingWeights_.beta[1] = 0.30;  // Distance stability (escape jammer)
    jammingWeights_.beta[2] = 0.10;  // Relative velocity
    jammingWeights_.beta[3] = 0.10;  // Heading alignment
    jammingWeights_.beta[4] = 0.25;  // Altitude stability (3D diversity)
    jammingWeights_.beta[5] = 0.15;  // Flight predictability
    
    currentWeights_ = normalWeights_;
    
    initialized_ = true;
    underJamming_ = false;
    handoffInProgress_ = false;
    lastAdaptation_ = ns3::Simulator::Now();
    
    NS_LOG_UNCOND("[CLC] Initialized for " << numNodes << " nodes");
}

void CrossLayerController::ConnectNs3Callbacks() {
    // Connect to PHY layer traces for SINR monitoring
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                         ns3::MakeCallback(&CrossLayerController::PhyRxCallback));
    
    // Connect to PHY TX for power monitoring
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                         ns3::MakeCallback(&CrossLayerController::PhyTxCallback));
    
    // Connect to MAC layer traces
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                         ns3::MakeCallback(&CrossLayerController::MacTxCallback));
    
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                         ns3::MakeCallback(&CrossLayerController::MacTxDropCallback));
    
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                         ns3::MakeCallback(&CrossLayerController::MacRxCallback));
    
    ns3::Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop",
                         ns3::MakeCallback(&CrossLayerController::MacRxDropCallback));
    
    NS_LOG_UNCOND("[CLC] Connected NS-3 PHY/MAC trace callbacks");
}

void CrossLayerController::StartPeriodicUpdates() {
    ns3::Simulator::Schedule(ns3::Seconds(adaptationInterval_), 
                             &CrossLayerController::PeriodicUpdate, this);
    NS_LOG_UNCOND("[CLC] Periodic updates started (interval: " << adaptationInterval_ << "s)");
}

// === PHY Layer Callbacks ===
void CrossLayerController::PhyRxCallback(std::string context, 
                                          ns3::Ptr<const ns3::Packet> packet,
                                          uint16_t channelFreqMhz, 
                                          ns3::WifiTxVector txVector,
                                          ns3::MpduInfo aMpdu, 
                                          ns3::SignalNoiseDbm signalNoise,
                                          uint16_t staId) {
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    double sinrDb = signalNoise.signal - signalNoise.noise;
    
    // Estimate PER from SINR (simplified model)
    double perEstimate = 0.0;
    if (sinrDb < 5.0) {
        perEstimate = 0.5;
    } else if (sinrDb < 10.0) {
        perEstimate = 0.3 * (10.0 - sinrDb) / 5.0;
    } else if (sinrDb < 15.0) {
        perEstimate = 0.1 * (15.0 - sinrDb) / 5.0;
    }
    
    Instance().UpdatePhyState(nodeId, sinrDb, perEstimate);
    Instance().RecordRxPower(nodeId, signalNoise.signal);
}

void CrossLayerController::PhyTxCallback(std::string context, 
                                          ns3::Ptr<const ns3::Packet> packet,
                                          double txPowerDbm) {
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    // Track TX power for power control feedback
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " TX at " << txPowerDbm << " dBm");
}

// === MAC Layer Callbacks ===
void CrossLayerController::MacTxCallback(std::string context, 
                                          ns3::Ptr<const ns3::Packet> packet) {
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    Instance().RecordTxAttempt(nodeId);
    Instance().RecordTxSuccess(nodeId);
}

void CrossLayerController::MacTxDropCallback(std::string context, 
                                              ns3::Ptr<const ns3::Packet> packet) {
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    Instance().RecordTxAttempt(nodeId);
    Instance().RecordTxFailure(nodeId);
    Instance().RecordCollision(nodeId);
}

void CrossLayerController::MacRxCallback(std::string context, 
                                          ns3::Ptr<const ns3::Packet> packet) {
    // Successful MAC reception - indicates good channel
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    // Update channel busy estimate
    auto& state = Instance().nodeStates_[nodeId];
    state.channelBusyRatio = 0.9 * state.channelBusyRatio + 0.1 * 0.5;  // Exponential average
}

void CrossLayerController::MacRxDropCallback(std::string context, 
                                              ns3::Ptr<const ns3::Packet> packet) {
    uint32_t nodeId = ExtractNodeIdFromContext(context);
    if (nodeId >= Instance().numNodes_) return;
    
    Instance().RecordPacketDrop(nodeId);
    
    // High drop rate indicates interference
    auto& state = Instance().nodeStates_[nodeId];
    state.channelBusyRatio = 0.9 * state.channelBusyRatio + 0.1 * 0.8;
}

// === Information Collection (UPWARD) ===
void CrossLayerController::UpdatePhyState(uint32_t nodeId, double sinrDb, double perEstimate) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    auto& state = nodeStates_[nodeId];
    
    // Exponential moving average for smoothing
    const double alpha = 0.3;
    state.avgSinr = (1 - alpha) * state.avgSinr + alpha * sinrDb;
    state.minSinr = std::min(state.minSinr, sinrDb);
    state.packetErrorRate = (1 - alpha) * state.packetErrorRate + alpha * perEstimate;
    state.phySamples++;
    state.lastPhyUpdate = ns3::Simulator::Now();
    
    // Update interference level (inverse of SINR normalized)
    state.interferenceLevel = std::max(0.0, std::min(1.0, (20.0 - sinrDb) / 25.0));
}

void CrossLayerController::RecordRxPower(uint32_t nodeId, double rxPowerDbm) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].rxPowerDbm = rxPowerDbm;
}

void CrossLayerController::RecordInterference(uint32_t nodeId, double interferenceDbm) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    // Convert interference power to normalized level
    double level = std::max(0.0, std::min(1.0, (interferenceDbm + 80) / 40.0));
    nodeStates_[nodeId].interferenceLevel = level;
}

void CrossLayerController::RecordTxAttempt(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].txAttempts++;
    nodeStates_[nodeId].lastMacUpdate = ns3::Simulator::Now();
}

void CrossLayerController::RecordTxSuccess(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].txSuccess++;
}

void CrossLayerController::RecordTxFailure(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].txFailures++;
}

void CrossLayerController::RecordCollision(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].collisionCount++;
}

void CrossLayerController::UpdateChannelBusy(uint32_t nodeId, double busyRatio) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].channelBusyRatio = busyRatio;
}

void CrossLayerController::RecordBackoff(uint32_t nodeId, uint32_t slots) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    auto& state = nodeStates_[nodeId];
    state.avgBackoffSlots = 0.9 * state.avgBackoffSlots + 0.1 * slots;
}

void CrossLayerController::RecordRouteChange(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    auto& state = nodeStates_[nodeId];
    state.routeChanges++;
    state.routeStability = std::max(0.0, state.routeStability - 0.1);
    state.lastNetUpdate = ns3::Simulator::Now();
}

void CrossLayerController::UpdateRouteMetrics(uint32_t nodeId, double avgHops) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].avgHopCount = avgHops;
}

void CrossLayerController::RecordRoutingOverhead(uint32_t nodeId, uint32_t bytes) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].routingOverhead += bytes;
}

void CrossLayerController::UpdateThroughput(uint32_t nodeId, double throughputMbps) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].flowThroughput = throughputMbps;
    nodeStates_[nodeId].lastAppUpdate = ns3::Simulator::Now();
}

void CrossLayerController::UpdateQueueState(uint32_t nodeId, double fillRatio) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].queueOccupancy = fillRatio;
}

void CrossLayerController::RecordPacketDrop(uint32_t nodeId) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    nodeStates_[nodeId].droppedPackets++;
}

void CrossLayerController::RecordDelay(uint32_t nodeId, double delaySeconds) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (nodeId >= numNodes_) return;
    
    auto& state = nodeStates_[nodeId];
    state.avgDelay = 0.9 * state.avgDelay + 0.1 * delaySeconds;
}

// === Control Commands (DOWNWARD) ===
void CrossLayerController::SetTxPower(uint32_t nodeId, double powerDbm) {
    if (nodeId >= numNodes_) return;
    
    ns3::Ptr<ns3::Node> node = fanetNodes.Get(nodeId);
    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
        ns3::Ptr<ns3::WifiNetDevice> wifiDev = 
            ns3::DynamicCast<ns3::WifiNetDevice>(node->GetDevice(i));
        if (wifiDev) {
            ns3::Ptr<ns3::WifiPhy> phy = wifiDev->GetPhy();
            if (phy) {
                phy->SetTxPowerStart(powerDbm);
                phy->SetTxPowerEnd(powerDbm);
                NS_LOG_DEBUG("[CLC] Node " << nodeId << " TX power set to " << powerDbm << " dBm");
            }
        }
    }
}

void CrossLayerController::RequestChannelSwitch(uint32_t channel) {
    NS_LOG_UNCOND("[CLC] Channel switch requested to channel " << channel);
    // In NS-3, channel switching would require PHY reconfiguration
    // This is handled by the handoff mechanism
}

void CrossLayerController::SetContentionWindow(uint32_t nodeId, uint32_t cwMin, uint32_t cwMax) {
    if (nodeId >= numNodes_) return;
    
    // Note: Modifying CW at runtime requires DCF/EDCA parameter access
    // This is a placeholder for the interface
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " CW set to [" << cwMin << ", " << cwMax << "]");
}

void CrossLayerController::SetQueuePriority(uint32_t nodeId, uint32_t priority) {
    if (nodeId >= numNodes_) return;
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " queue priority set to " << priority);
}

void CrossLayerController::SetRoutePriority(uint32_t srcNode, uint32_t dstNode, double priority) {
    NS_LOG_DEBUG("[CLC] Route " << srcNode << " -> " << dstNode << " priority: " << priority);
}

void CrossLayerController::TriggerRouteRefresh(uint32_t nodeId) {
    if (nodeId >= numNodes_) return;
    
    ns3::Ptr<ns3::Ipv4> ipv4 = fanetNodes.Get(nodeId)->GetObject<ns3::Ipv4>();
    if (!ipv4) return;
    
    ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
    if (!rp) return;
    
    for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
        rp->NotifyInterfaceUp(j);
    }
    
    NS_LOG_DEBUG("[CLC] Route refresh triggered for node " << nodeId);
}

void CrossLayerController::AvoidNode(uint32_t nodeToAvoid) {
    NS_LOG_UNCOND("[CLC] Avoiding node " << nodeToAvoid << " in routing decisions");
    // This would integrate with LMSS scoring to penalize routes through this node
}

void CrossLayerController::SetFlowRate(uint32_t nodeId, double rateMbps) {
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " flow rate set to " << rateMbps << " Mbps");
}

void CrossLayerController::PauseFlow(uint32_t nodeId) {
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " flow paused");
}

void CrossLayerController::ResumeFlow(uint32_t nodeId) {
    NS_LOG_DEBUG("[CLC] Node " << nodeId << " flow resumed");
}

// === Adaptive LMSS Weights ===
void CrossLayerController::SetFixedWeights(const LmssWeights& weights) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    currentWeights_ = weights;
    fixedWeightsMode_ = true;
    NS_LOG_UNCOND("[CLC] Fixed LMSS weights set - adaptive tuning DISABLED");
    NS_LOG_UNCOND("  Weights: [" << weights.beta[0] << ", " << weights.beta[1] << ", "
                  << weights.beta[2] << ", " << weights.beta[3] << ", "
                  << weights.beta[4] << ", " << weights.beta[5] << "]");
}

void CrossLayerController::AdaptLmssWeights() {
    // Skip adaptation if using fixed weights (for protocol emulation)
    if (fixedWeightsMode_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    // Calculate network-wide averages
    double avgSinr = 0, avgPer = 0, avgMacEff = 0;
    uint32_t jammingNodes = 0;
    
    for (const auto& [nodeId, state] : nodeStates_) {
        avgSinr += state.avgSinr;
        avgPer += state.packetErrorRate;
        avgMacEff += state.macEfficiency;
        if (state.isUnderJamming()) jammingNodes++;
    }
    
    if (numNodes_ > 0) {
        avgSinr /= numNodes_;
        avgPer /= numNodes_;
        avgMacEff /= numNodes_;
    }
    
    double jammingRatio = static_cast<double>(jammingNodes) / numNodes_;
    bool wasUnderJamming = underJamming_;
    
    // Detect jamming condition
    underJamming_ = (avgSinr < jammingSinrThreshold_) || 
                    (avgPer > jammingPerThreshold_) ||
                    (jammingRatio > jammingNodeRatioThreshold_);
    
    // Smooth transition between weight sets
    if (underJamming_) {
        // Transition to jamming weights
        const double transitionRate = 0.3;
        for (int i = 0; i < 6; ++i) {
            currentWeights_.beta[i] = (1 - transitionRate) * currentWeights_.beta[i] + 
                                       transitionRate * jammingWeights_.beta[i];
        }
        
        if (!wasUnderJamming) {
            NS_LOG_UNCOND("[CLC] 🚨 JAMMING DETECTED - Adapting LMSS weights");
            NS_LOG_UNCOND("  Avg SINR: " << avgSinr << " dB, Avg PER: " << avgPer);
            NS_LOG_UNCOND("  Nodes affected: " << jammingNodes << "/" << numNodes_);
        }
    } else {
        // Transition back to normal weights
        const double transitionRate = 0.1;  // Slower recovery
        for (int i = 0; i < 6; ++i) {
            currentWeights_.beta[i] = (1 - transitionRate) * currentWeights_.beta[i] + 
                                       transitionRate * normalWeights_.beta[i];
        }
        
        if (wasUnderJamming) {
            NS_LOG_UNCOND("[CLC] ✅ Jamming cleared - Restoring normal LMSS weights");
        }
    }
    
    currentWeights_.normalize();
    lastAdaptation_ = ns3::Simulator::Now();
}

// === State Queries ===
const CrossLayerState& CrossLayerController::GetNodeState(uint32_t nodeId) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    static CrossLayerState emptyState;
    
    auto it = nodeStates_.find(nodeId);
    if (it != nodeStates_.end()) {
        return it->second;
    }
    return emptyState;
}

NetworkState CrossLayerController::GetNetworkState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    NetworkState ns;
    ns.totalNodes = numNodes_;
    ns.timestamp = ns3::Simulator::Now();
    
    double sumSinr = 0, sumPer = 0, sumMacEff = 0, sumTput = 0;
    
    for (const auto& [nodeId, state] : nodeStates_) {
        sumSinr += state.avgSinr;
        sumPer += state.packetErrorRate;
        sumMacEff += state.macEfficiency;
        sumTput += state.flowThroughput;
        if (state.isUnderJamming()) ns.nodesUnderJamming++;
    }
    
    if (numNodes_ > 0) {
        ns.avgSinr = sumSinr / numNodes_;
        ns.avgPer = sumPer / numNodes_;
        ns.avgMacEfficiency = sumMacEff / numNodes_;
        ns.avgThroughput = sumTput / numNodes_;
    }
    
    ns.networkHealth = 1.0 - (static_cast<double>(ns.nodesUnderJamming) / numNodes_);
    
    return ns;
}

bool CrossLayerController::IsNetworkUnderJamming() const {
    return underJamming_;
}

double CrossLayerController::GetNetworkHealth() const {
    auto ns = GetNetworkState();
    return ns.networkHealth;
}

// === Handoff Integration ===
bool CrossLayerController::ShouldTriggerHandoff() const {
    if (handoffInProgress_) return false;
    
    auto ns = GetNetworkState();
    
    // Trigger if:
    // 1. More than 30% of nodes under jamming, OR
    // 2. Average SINR critically low, OR
    // 3. Network health below threshold
    return (static_cast<double>(ns.nodesUnderJamming) / ns.totalNodes > 0.3) ||
           (ns.avgSinr < 5.0) ||
           (ns.networkHealth < 0.4);
}

void CrossLayerController::NotifyHandoffStarted() {
    handoffInProgress_ = true;
    NS_LOG_UNCOND("[CLC] Handoff started - suspending weight adaptation");
}

void CrossLayerController::NotifyHandoffComplete() {
    handoffInProgress_ = false;
    underJamming_ = false;  // Reset jamming state
    currentWeights_ = normalWeights_;  // Reset to normal weights
    
    // Reset node states after handoff
    for (auto& [nodeId, state] : nodeStates_) {
        state.minSinr = 20.0;  // Reset minimum
        state.packetErrorRate = 0.0;
        state.interferenceLevel = 0.0;
        state.collisionCount = 0;
        state.txFailures = 0;
        state.routeStability = 1.0;
    }
    
    NS_LOG_UNCOND("[CLC] Handoff complete - states reset, normal weights restored");
}

// === Periodic Update ===
void CrossLayerController::PeriodicUpdate() {
    ComputeDerivedMetrics();
    CheckJammingConditions();
    UpdateMacEfficiency();
    AdaptLmssWeights();
    
    if (verboseLogging_) {
        PrintNetworkSummary();
    }
    
    // Schedule next update
    ns3::Simulator::Schedule(ns3::Seconds(adaptationInterval_), 
                             &CrossLayerController::PeriodicUpdate, this);
}

void CrossLayerController::ComputeDerivedMetrics() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    for (auto& [nodeId, state] : nodeStates_) {
        // Gradually recover route stability if no changes
        double timeSinceLastChange = (ns3::Simulator::Now() - state.lastNetUpdate).GetSeconds();
        if (timeSinceLastChange > 5.0) {
            state.routeStability = std::min(1.0, state.routeStability + 0.05);
        }
    }
}

void CrossLayerController::CheckJammingConditions() {
    // Already handled in AdaptLmssWeights
}

void CrossLayerController::UpdateMacEfficiency() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    for (auto& [nodeId, state] : nodeStates_) {
        if (state.txAttempts > 0) {
            state.macEfficiency = static_cast<double>(state.txSuccess) / state.txAttempts;
        }
    }
}

// === Logging ===
void CrossLayerController::PrintNodeState(uint32_t nodeId) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    auto it = nodeStates_.find(nodeId);
    if (it == nodeStates_.end()) return;
    
    const auto& s = it->second;
    NS_LOG_UNCOND("=== Node " << nodeId << " Cross-Layer State ===");
    NS_LOG_UNCOND("  PHY: SINR=" << std::fixed << std::setprecision(1) << s.avgSinr 
                  << "dB, PER=" << std::setprecision(2) << s.packetErrorRate
                  << ", Interference=" << s.interferenceLevel);
    NS_LOG_UNCOND("  MAC: Efficiency=" << s.macEfficiency 
                  << ", Collisions=" << s.collisionCount);
    NS_LOG_UNCOND("  NET: RouteStability=" << s.routeStability 
                  << ", Changes=" << s.routeChanges);
    NS_LOG_UNCOND("  APP: Throughput=" << s.flowThroughput 
                  << "Mbps, Delay=" << s.avgDelay << "s");
    NS_LOG_UNCOND("  Health: " << s.overallHealth() 
                  << ", UnderJamming=" << (s.isUnderJamming() ? "YES" : "NO"));
}

void CrossLayerController::PrintNetworkSummary() const {
    auto ns = GetNetworkState();
    
    NS_LOG_UNCOND("\n=== Network Summary @ " << ns.timestamp.GetSeconds() << "s ===");
    NS_LOG_UNCOND("  Nodes: " << ns.totalNodes << ", Under Jamming: " << ns.nodesUnderJamming);
    NS_LOG_UNCOND("  Avg SINR: " << std::fixed << std::setprecision(1) << ns.avgSinr << " dB");
    NS_LOG_UNCOND("  Avg PER: " << std::setprecision(2) << ns.avgPer);
    NS_LOG_UNCOND("  MAC Efficiency: " << ns.avgMacEfficiency);
    NS_LOG_UNCOND("  Network Health: " << ns.networkHealth);
    NS_LOG_UNCOND("  LMSS Weights: [" << std::setprecision(2)
                  << currentWeights_.beta[0] << ", " << currentWeights_.beta[1] << ", "
                  << currentWeights_.beta[2] << ", " << currentWeights_.beta[3] << ", "
                  << currentWeights_.beta[4] << ", " << currentWeights_.beta[5] << "]");
}

} // namespace myfanet
