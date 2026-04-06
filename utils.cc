/*
 * utils.cc - Utility Function Implementations
 */

#include "utils.h"

namespace myfanet {

double GetDistance(ns3::Ptr<ns3::Node> n1, ns3::Ptr<ns3::Node> n2) {
    ns3::Ptr<ns3::MobilityModel> m1 = n1->GetObject<ns3::MobilityModel>();
    ns3::Ptr<ns3::MobilityModel> m2 = n2->GetObject<ns3::MobilityModel>();
    return m1->GetDistanceFrom(m2);
}

ns3::Vector GetNodePosition(ns3::Ptr<ns3::Node> node) {
    ns3::Ptr<ns3::MobilityModel> mm = node->GetObject<ns3::MobilityModel>();
    return mm ? mm->GetPosition() : ns3::Vector(0, 0, 0);
}

ns3::Vector GetNodeVelocity(ns3::Ptr<ns3::Node> node) {
    // Use base MobilityModel class - works with ALL mobility models
    // including GaussMarkovMobilityModel, ConstantVelocityMobilityModel, etc.
    ns3::Ptr<ns3::MobilityModel> mm = node->GetObject<ns3::MobilityModel>();
    return mm ? mm->GetVelocity() : ns3::Vector(0, 0, 0);
}

} // namespace myfanet
