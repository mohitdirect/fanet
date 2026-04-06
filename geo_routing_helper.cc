/*
 * geo_routing_helper.cc - Helper for GeoRoutingProtocol
 */

#include "geo_routing_helper.h"
#include "ns3/ipv4-list-routing.h"

namespace myfanet {

GeoRoutingHelper::GeoRoutingHelper() {
}

GeoRoutingHelper* GeoRoutingHelper::Copy(void) const {
    return new GeoRoutingHelper(*this);
}

ns3::Ptr<ns3::Ipv4RoutingProtocol> GeoRoutingHelper::Create(ns3::Ptr<ns3::Node> node) const {
    return ns3::CreateObject<GeoRoutingProtocol>();
}

} // namespace myfanet
