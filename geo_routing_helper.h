/*
 * geo_routing_helper.h - Helper for GeoRoutingProtocol
 */

#ifndef GEO_ROUTING_HELPER_H
#define GEO_ROUTING_HELPER_H

#include "ns3/ipv4-routing-helper.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "geo_routing_protocol.h"

namespace myfanet {

class GeoRoutingHelper : public ns3::Ipv4RoutingHelper {
public:
    GeoRoutingHelper();

    // Returns a copy of the helper
    virtual GeoRoutingHelper* Copy(void) const override;

    // Creates the routing protocol object
    virtual ns3::Ptr<ns3::Ipv4RoutingProtocol> Create(ns3::Ptr<ns3::Node> node) const override;
};

} // namespace myfanet

#endif // GEO_ROUTING_HELPER_H
