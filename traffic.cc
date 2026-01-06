/*
 * traffic.cc - Traffic Pattern Setup Implementation
 */

#include "traffic.h"
#include "config.h"
#include "types.h"
#include "ns3/log.h"
#include <random>
#include <algorithm>

NS_LOG_COMPONENT_DEFINE("Traffic");

namespace myfanet {

// Setup aircraft communication pairs with ACKs
void SetupAircraftCommunication(uint32_t numNodes, double dataRate, double ackRate)
{
    aircraftPairs.clear();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> numReceiversDist(2, 3);  // 2-3 receivers per sender
    
    // Create pairs: each node is a sender to 2-3 random nodes
    for (uint32_t sender = 0; sender < numNodes; ++sender) {
        AircraftCommPair pair;
        pair.sender = sender;
        
        std::vector<uint32_t> candidates;
        for (uint32_t i = 0; i < numNodes; ++i) {
            if (i != sender) candidates.push_back(i);
        }
        
        // Shuffle and select 2-3 random receivers
        std::shuffle(candidates.begin(), candidates.end(), gen);
        uint32_t numReceivers = numReceiversDist(gen);
        numReceivers = std::min(numReceivers, (uint32_t)candidates.size());
        
        for (uint32_t r = 0; r < numReceivers; ++r) {
            pair.receivers.push_back(candidates[r]);
        }
        
        aircraftPairs.push_back(pair);
    }
    
    NS_LOG_UNCOND("\n=== AIRCRAFT COMMUNICATION SETUP (with ACKs) ===");
    NS_LOG_UNCOND("Nodes: " << numNodes);
    NS_LOG_UNCOND("Data Rate: " << dataRate << " Kbps");
    NS_LOG_UNCOND("ACK Rate: " << ackRate << " Kbps\n");
    
    for (const auto& pair : aircraftPairs) {
        NS_LOG_UNCOND("Aircraft-" << pair.sender << " → [");
        for (uint32_t r = 0; r < pair.receivers.size(); ++r) {
            NS_LOG_UNCOND("Aircraft-" << pair.receivers[r]);
            if (r < pair.receivers.size() - 1) NS_LOG_UNCOND(", ");
        }
        NS_LOG_UNCOND("] (+ ACKs back)");
    }
    NS_LOG_UNCOND("Total flows (data + ACKs): " << (numNodes * 2 * 2.5) << "\n");
}

} // namespace myfanet
