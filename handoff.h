/*
 * handoff.h - Handoff Execution and PGH Logic
 */

#ifndef MYFANET_HANDOFF_H
#define MYFANET_HANDOFF_H

#include "ns3/core-module.h"
#include <string>

namespace myfanet {

// Trigger routing refresh after handoff
void TriggerRoutingRefresh();

// Route caching for faster recovery
void CacheCurrentRoutes();
void RestoreCachedRoutes();

// Execute the simulated frequency handoff
void ExecuteHandoff(const std::string& tag);

// Main PGH-LMSS logic update loop
void UpdatePghLogic(ns3::Time interval);

// Phase transition helpers
void TransitionToJammingPhase();
void TransitionToRecoveryPhase();

// Cluster formation and management
void FormClusters();
void UpdateClusterMembership();
uint32_t GetClusterHead(uint32_t nodeId);

} // namespace myfanet

#endif // MYFANET_HANDOFF_H
