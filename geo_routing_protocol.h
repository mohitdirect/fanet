/*
 * geo_routing_protocol.h - Geographic Routing Protocol Wrapper
 * 
 * A lightweight Ipv4RoutingProtocol that:
 * 1. Consults GeoRouter for next-hop decisions
 * 2. falls back to a backup protocol (OLSR) if needed
 * 3. Handles packet forwarding based on geographic position
 */

#ifndef GEO_ROUTING_PROTOCOL_H
#define GEO_ROUTING_PROTOCOL_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/callback.h"
#include "ns3/ipv4-header.h"
#include "geo_routing.h"

namespace myfanet {

class GeoRoutingProtocol : public ns3::Ipv4RoutingProtocol {
public:
    static ns3::TypeId GetTypeId(void);

    GeoRoutingProtocol();
    virtual ~GeoRoutingProtocol();

    // Set the backup routing protocol (e.g., OLSR)
    void SetBackupProtocol(ns3::Ptr<ns3::Ipv4RoutingProtocol> backup);

    // Ipv4RoutingProtocol methods
    virtual ns3::Ptr<ns3::Ipv4Route> RouteOutput(ns3::Ptr<ns3::Packet> p,
                                                 const ns3::Ipv4Header &header,
                                                 ns3::Ptr<ns3::NetDevice> oif,
                                                 ns3::Socket::SocketErrno &sockerr) override;

    virtual bool RouteInput(ns3::Ptr<const ns3::Packet> p,
                            const ns3::Ipv4Header &header,
                            ns3::Ptr<const ns3::NetDevice> idev,
                            const ns3::Ipv4RoutingProtocol::UnicastForwardCallback &ucb,
                            const ns3::Ipv4RoutingProtocol::MulticastForwardCallback &mcb,
                            const ns3::Ipv4RoutingProtocol::LocalDeliverCallback &lcb,
                            const ns3::Ipv4RoutingProtocol::ErrorCallback &ecb) override;

    virtual void NotifyInterfaceUp(uint32_t interface) override;
    virtual void NotifyInterfaceDown(uint32_t interface) override;
    virtual void NotifyAddAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) override;
    virtual void NotifyRemoveAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) override;
    virtual void SetIpv4(ns3::Ptr<ns3::Ipv4> ipv4) override;
    virtual void PrintRoutingTable(ns3::Ptr<ns3::OutputStreamWrapper> stream, ns3::Time::Unit unit = ns3::Time::S) const override;

private:
    ns3::Ptr<ns3::Ipv4> m_ipv4;
    ns3::Ptr<ns3::Ipv4RoutingProtocol> m_backupProtocol;
    
    // Internal helper to get Ipv4Route from next hop node ID
    ns3::Ptr<ns3::Ipv4Route> GetRouteToNextHop(uint32_t nextHopId, ns3::Ipv4Address dst);
};

} // namespace myfanet

#endif // GEO_ROUTING_PROTOCOL_H
