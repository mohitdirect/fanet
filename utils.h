/*
 * utils.h - Utility Functions for FANET Simulation
 */

#ifndef MYFANET_UTILS_H
#define MYFANET_UTILS_H

#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

namespace myfanet {

// Calculate distance between two nodes
double GetDistance(ns3::Ptr<ns3::Node> n1, ns3::Ptr<ns3::Node> n2);

// Get node position vector
ns3::Vector GetNodePosition(ns3::Ptr<ns3::Node> node);

// Get node velocity vector
ns3::Vector GetNodeVelocity(ns3::Ptr<ns3::Node> node);

} // namespace myfanet

#endif // MYFANET_UTILS_H
