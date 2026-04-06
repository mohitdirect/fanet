/*
 * cross_layer_controller.h - Centralized Cross-Layer Controller for FANET
 * 
 * Provides bidirectional information sharing between PHY, MAC, Network,
 * and Application layers for improved performance under jamming.
 * 
 * Architecture:
 *   - Singleton pattern for centralized decision making
 *   - Collects metrics from all layers via NS-3 callbacks
 *   - Issues control commands downward to all layers
 *   - Manages adaptive LMSS weight optimization
 */

#ifndef MYFANET_CROSS_LAYER_CONTROLLER_H
#define MYFANET_CROSS_LAYER_CONTROLLER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include <map>
#include <vector>
#include <functional>
#include <mutex>

namespace myfanet {

// Forward declarations
struct NodeMetrics;

// === Cross-Layer State Structure ===
// Aggregates information from all network layers for each node
struct CrossLayerState {
    // === PHY Layer Metrics ===
    double avgSinr = 20.0;              // Average SINR in dB
    double minSinr = 20.0;              // Minimum SINR observed
    double packetErrorRate = 0.0;       // Current PER
    double interferenceLevel = 0.0;     // 0-1 normalized interference
    double rxPowerDbm = -70.0;          // Received power
    uint32_t phySamples = 0;            // Number of PHY samples
    
    // === MAC Layer Metrics ===
    double channelBusyRatio = 0.0;      // Fraction of time channel is busy
    uint32_t txAttempts = 0;            // Transmission attempts
    uint32_t txSuccess = 0;             // Successful transmissions
    uint32_t txFailures = 0;            // Failed transmissions (collisions/timeout)
    uint32_t collisionCount = 0;        // Detected collisions
    double avgBackoffSlots = 0.0;       // Average backoff slots used
    double macEfficiency = 1.0;         // txSuccess / txAttempts
    
    // === Network Layer Metrics ===
    double routeStability = 1.0;        // Route change frequency (1 = stable)
    uint32_t routeChanges = 0;          // Number of route changes
    double avgHopCount = 1.0;           // Average hops to destinations
    uint32_t routingOverhead = 0;       // Routing control bytes sent
    
    // === Application Layer Metrics ===
    double flowThroughput = 0.0;        // Achieved throughput (Mbps)
    double queueOccupancy = 0.0;        // Queue fill ratio (0-1)
    uint32_t droppedPackets = 0;        // Application-level drops
    double avgDelay = 0.0;              // Average end-to-end delay
    
    // === Timestamps ===
    ns3::Time lastPhyUpdate;
    ns3::Time lastMacUpdate;
    ns3::Time lastNetUpdate;
    ns3::Time lastAppUpdate;
    
    // === Derived Metrics ===
    double overallHealth() const {
        // Weighted combination of layer-specific health indicators
        double phyHealth = (avgSinr - 5.0) / 20.0;  // 5dB=0, 25dB=1
        double macHealth = macEfficiency;
        double netHealth = routeStability;
        double appHealth = (flowThroughput > 0) ? std::min(1.0, flowThroughput / 0.15) : 0.5;
        
        return std::max(0.0, std::min(1.0, 
            0.3 * phyHealth + 0.25 * macHealth + 0.25 * netHealth + 0.2 * appHealth));
    }
    
    bool isUnderJamming() const {
        return avgSinr < 10.0 || packetErrorRate > 0.2 || interferenceLevel > 0.5;
    }
};

// === Network-Wide State Snapshot ===
struct NetworkState {
    double avgSinr = 20.0;
    double avgPer = 0.0;
    double avgMacEfficiency = 1.0;
    double avgThroughput = 0.0;
    uint32_t nodesUnderJamming = 0;
    uint32_t totalNodes = 0;
    double networkHealth = 1.0;
    ns3::Time timestamp;
};

// === LMSS Weight Configuration ===
struct LmssWeights {
    double beta[6] = {0.15, 0.20, 0.15, 0.15, 0.15, 0.20};
    
    // Named accessors for clarity
    double& linkDuration() { return beta[0]; }
    double& distanceStability() { return beta[1]; }
    double& relativeVelocity() { return beta[2]; }
    double& headingAlignment() { return beta[3]; }
    double& altitudeStability() { return beta[4]; }
    double& flightPredictability() { return beta[5]; }
    
    void normalize() {
        double sum = 0;
        for (int i = 0; i < 6; ++i) sum += beta[i];
        if (sum > 0) {
            for (int i = 0; i < 6; ++i) beta[i] /= sum;
        }
    }
};

// === Cross-Layer Controller (Singleton) ===
class CrossLayerController {
public:
    // === Singleton Access ===
    static CrossLayerController& Instance();
    
    // === Initialization ===
    void Initialize(uint32_t numNodes);
    void ConnectNs3Callbacks();
    void StartPeriodicUpdates();
    
    // === Information Collection (UPWARD flow) ===
    // PHY Layer
    void UpdatePhyState(uint32_t nodeId, double sinrDb, double perEstimate);
    void RecordRxPower(uint32_t nodeId, double rxPowerDbm);
    void RecordInterference(uint32_t nodeId, double interferenceDbm);
    
    // MAC Layer
    void RecordTxAttempt(uint32_t nodeId);
    void RecordTxSuccess(uint32_t nodeId);
    void RecordTxFailure(uint32_t nodeId);
    void RecordCollision(uint32_t nodeId);
    void UpdateChannelBusy(uint32_t nodeId, double busyRatio);
    void RecordBackoff(uint32_t nodeId, uint32_t slots);
    
    // Network Layer
    void RecordRouteChange(uint32_t nodeId);
    void UpdateRouteMetrics(uint32_t nodeId, double avgHops);
    void RecordRoutingOverhead(uint32_t nodeId, uint32_t bytes);
    
    // Application Layer
    void UpdateThroughput(uint32_t nodeId, double throughputMbps);
    void UpdateQueueState(uint32_t nodeId, double fillRatio);
    void RecordPacketDrop(uint32_t nodeId);
    void RecordDelay(uint32_t nodeId, double delaySeconds);
    
    // === Control Commands (DOWNWARD flow) ===
    // PHY Layer Controls
    void SetTxPower(uint32_t nodeId, double powerDbm);
    void RequestChannelSwitch(uint32_t channel);
    
    // MAC Layer Controls
    void SetContentionWindow(uint32_t nodeId, uint32_t cwMin, uint32_t cwMax);
    void SetQueuePriority(uint32_t nodeId, uint32_t priority);
    
    // Network Layer Controls
    void SetRoutePriority(uint32_t srcNode, uint32_t dstNode, double priority);
    void TriggerRouteRefresh(uint32_t nodeId);
    void AvoidNode(uint32_t nodeToAvoid);  // Mark node as unreliable relay
    
    // Application Layer Controls
    void SetFlowRate(uint32_t nodeId, double rateMbps);
    void PauseFlow(uint32_t nodeId);
    void ResumeFlow(uint32_t nodeId);
    
    // === Adaptive LMSS Weights ===
    const LmssWeights& GetCurrentWeights() const { return currentWeights_; }
    void AdaptLmssWeights();
    double GetAdaptiveBeta(int component) const { return currentWeights_.beta[component]; }
    
    // === Fixed Weights for Protocol Emulation ===
    // When fixed weights are set, AdaptLmssWeights() becomes a no-op
    void SetFixedWeights(const LmssWeights& weights);
    bool IsUsingFixedWeights() const { return fixedWeightsMode_; }
    
    // === State Queries ===
    const CrossLayerState& GetNodeState(uint32_t nodeId) const;
    NetworkState GetNetworkState() const;
    bool IsNetworkUnderJamming() const;
    double GetNetworkHealth() const;
    
    // === Handoff Integration ===
    bool ShouldTriggerHandoff() const;
    void NotifyHandoffStarted();
    void NotifyHandoffComplete();
    
    // === Logging and Debug ===
    void PrintNodeState(uint32_t nodeId) const;
    void PrintNetworkSummary() const;
    void EnableVerboseLogging(bool enable) { verboseLogging_ = enable; }
    
    // === NS-3 Trace Callbacks (static for NS-3 compatibility) ===
    static void PhyRxCallback(std::string context, ns3::Ptr<const ns3::Packet> packet,
                              uint16_t channelFreqMhz, ns3::WifiTxVector txVector,
                              ns3::MpduInfo aMpdu, ns3::SignalNoiseDbm signalNoise,
                              uint16_t staId);
    
    static void PhyTxCallback(std::string context, ns3::Ptr<const ns3::Packet> packet,
                              double txPowerDbm);
    
    static void MacTxCallback(std::string context, ns3::Ptr<const ns3::Packet> packet);
    static void MacTxDropCallback(std::string context, ns3::Ptr<const ns3::Packet> packet);
    static void MacRxCallback(std::string context, ns3::Ptr<const ns3::Packet> packet);
    static void MacRxDropCallback(std::string context, ns3::Ptr<const ns3::Packet> packet);
    
private:
    CrossLayerController() = default;
    ~CrossLayerController() = default;
    CrossLayerController(const CrossLayerController&) = delete;
    CrossLayerController& operator=(const CrossLayerController&) = delete;
    
    // === Internal Update Functions ===
    void PeriodicUpdate();
    void ComputeDerivedMetrics();
    void CheckJammingConditions();
    void UpdateMacEfficiency();
    
    // === State Storage ===
    std::map<uint32_t, CrossLayerState> nodeStates_;
    LmssWeights currentWeights_;
    LmssWeights normalWeights_;      // Weights for normal operation
    LmssWeights jammingWeights_;     // Weights optimized for jamming
    
    // === Network-Wide State ===
    uint32_t numNodes_ = 0;
    bool initialized_ = false;
    bool underJamming_ = false;
    bool handoffInProgress_ = false;
    bool fixedWeightsMode_ = false;   // When true, AdaptLmssWeights is disabled
    ns3::Time lastAdaptation_;
    
    // === Configuration ===
    double adaptationInterval_ = 1.0;   // Seconds between weight adaptations
    double jammingSinrThreshold_ = 10.0;
    double jammingPerThreshold_ = 0.2;
    double jammingNodeRatioThreshold_ = 0.3;  // 30% of nodes affected
    bool verboseLogging_ = false;
    
    // === Thread Safety (for potential parallel simulation) ===
    mutable std::mutex stateMutex_;
    
    // === Static instance for singleton ===
    static CrossLayerController* instance_;
};

// === Inline Helper to Extract Node ID from NS-3 Context String ===
inline uint32_t ExtractNodeIdFromContext(const std::string& context) {
    // Context format: /NodeList/X/DeviceList/Y/...
    size_t pos = context.find("/NodeList/");
    if (pos == std::string::npos) return UINT32_MAX;
    
    size_t start = pos + 10;
    size_t end = context.find("/", start);
    if (end == std::string::npos) return UINT32_MAX;
    
    try {
        return std::stoul(context.substr(start, end - start));
    } catch (...) {
        return UINT32_MAX;
    }
}

} // namespace myfanet

#endif // MYFANET_CROSS_LAYER_CONTROLLER_H
