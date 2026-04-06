/*
 * geo_routing_protocol.cc - Geographic Routing Protocol Implementation
 */

#include "geo_routing_protocol.h"
#include "ns3/log.h"
#include "ns3/ipv4-route.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

NS_LOG_COMPONENT_DEFINE("GeoRoutingProtocol");

namespace myfanet {

NS_OBJECT_ENSURE_REGISTERED(GeoRoutingProtocol);

ns3::TypeId GeoRoutingProtocol::GetTypeId(void) {
    static ns3::TypeId tid = ns3::TypeId("myfanet::GeoRoutingProtocol")
        .SetParent<ns3::Ipv4RoutingProtocol>()
        .SetGroupName("Internet")
        .AddConstructor<GeoRoutingProtocol>();
    return tid;
}

GeoRoutingProtocol::GeoRoutingProtocol() : m_ipv4(0), m_backupProtocol(0) {
}

GeoRoutingProtocol::~GeoRoutingProtocol() {
}

void GeoRoutingProtocol::SetBackupProtocol(ns3::Ptr<ns3::Ipv4RoutingProtocol> backup) {
    m_backupProtocol = backup;
}

void GeoRoutingProtocol::SetIpv4(ns3::Ptr<ns3::Ipv4> ipv4) {
    m_ipv4 = ipv4;
    if (m_backupProtocol) {
        m_backupProtocol->SetIpv4(ipv4);
    }
}

// Helper to extract Node ID from IP (Assuming 10.1.1.X addressing)
// Node ID = Last octet - 1 (e.g., 10.1.1.1 -> Node 0)
// This is specific to the current simulation setup
static uint32_t GetNodeIdFromIp(ns3::Ipv4Address addr) {
    if (addr.IsBroadcast()) return UINT32_MAX;
    
    uint8_t buf[4];
    addr.Serialize(buf);
    
    // Check if it's in our 10.1.1.x subnet
    if (buf[0] == 10 && buf[1] == 1 && buf[2] == 1) {
        return buf[3] - 1;
    }
    return UINT32_MAX;
}

ns3::Ptr<ns3::Ipv4Route> GeoRoutingProtocol::RouteOutput(ns3::Ptr<ns3::Packet> p,
                                             const ns3::Ipv4Header &header,
                                             ns3::Ptr<ns3::NetDevice> oif,
                                             ns3::Socket::SocketErrno &sockerr) {
    NS_LOG_FUNCTION(this << p << header.GetDestination() << oif);

    // 1. Get Source and Destination Node IDs
    uint32_t srcId = m_ipv4->GetObject<ns3::Node>()->GetId();
    uint32_t dstId = GetNodeIdFromIp(header.GetDestination());

    // 2. If valid destination for GeoRouter
    if (dstId != UINT32_MAX) {
        // Consult GeoRouter
        uint32_t nextHopId = GeoRouter::Instance().GetHierarchicalNextHop(srcId, dstId);

        if (nextHopId != UINT32_MAX && nextHopId != srcId) {
            NS_LOG_LOGIC("GeoRouter found next hop: " << nextHopId << " for dst: " << dstId);
            return GetRouteToNextHop(nextHopId, header.GetDestination());
        }
    }

    // 3. Fallback to Backup Protocol (OLSR)
    if (m_backupProtocol) {
        return m_backupProtocol->RouteOutput(p, header, oif, sockerr);
    }

    sockerr = ns3::Socket::ERROR_NOROUTETOHOST;
    return 0;
}

bool GeoRoutingProtocol::RouteInput(ns3::Ptr<const ns3::Packet> p,
                        const ns3::Ipv4Header &header,
                        ns3::Ptr<const ns3::NetDevice> idev,
                        const ns3::Ipv4RoutingProtocol::UnicastForwardCallback &ucb,
                        const ns3::Ipv4RoutingProtocol::MulticastForwardCallback &mcb,
                        const ns3::Ipv4RoutingProtocol::LocalDeliverCallback &lcb,
                        const ns3::Ipv4RoutingProtocol::ErrorCallback &ecb) {
    NS_LOG_FUNCTION(this << p << header.GetDestination() << idev);

    // 1. Check for local delivery
    if (m_ipv4->IsDestinationAddress(header.GetDestination(), idev->GetIfIndex())) {
        lcb(p, header, idev->GetIfIndex());
        return true;
    }

    // 2. Multicast/Broadcast - defer to backup
    if (header.GetDestination().IsMulticast() || header.GetDestination().IsBroadcast()) {
        if (m_backupProtocol) {
            return m_backupProtocol->RouteInput(p, header, idev, ucb, mcb, lcb, ecb);
        }
        return false;
    }

    // 3. Unicast Forwarding
    uint32_t myId = m_ipv4->GetObject<ns3::Node>()->GetId();
    uint32_t dstId = GetNodeIdFromIp(header.GetDestination());

    if (dstId != UINT32_MAX) {
        // Consult GeoRouter
        uint32_t nextHopId = GeoRouter::Instance().GetHierarchicalNextHop(myId, dstId);

        if (nextHopId != UINT32_MAX && nextHopId != myId) {
            NS_LOG_LOGIC("GeoRouter forwarding: " << myId << " -> " << nextHopId << " (dst: " << dstId << ")");
            
            ns3::Ptr<ns3::Ipv4Route> route = GetRouteToNextHop(nextHopId, header.GetDestination());
            if (route) {
                ucb(route, p, header);
                return true;
            }
        }
    }

    // 4. Fallback
    if (m_backupProtocol) {
        return m_backupProtocol->RouteInput(p, header, idev, ucb, mcb, lcb, ecb);
    }

    return false;
}

ns3::Ptr<ns3::Ipv4Route> GeoRoutingProtocol::GetRouteToNextHop(uint32_t nextHopId, ns3::Ipv4Address dst) {
    // We need to find the interface to reach the next hop.
    // In this simulation, there's usually only one WiFi interface (index 1).
    // Loopback is 0.
    
    // We can iterate interfaces to find one within subnet? 
    // Simplified: Assume Interface 1 is the wifi interface.
    
    // We need the next hop's IP address
    // Assuming 10.1.1.(nextHopId + 1)
    ns3::Ipv4Address nextHopIp( (10 << 24) | (1 << 16) | (1 << 8) | (nextHopId + 1) );
    
    ns3::Ptr<ns3::Ipv4Route> route = ns3::Create<ns3::Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHopIp);
    route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal()); // Interface 1
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    
    return route;
}

void GeoRoutingProtocol::NotifyInterfaceUp(uint32_t interface) {
    if (m_backupProtocol) m_backupProtocol->NotifyInterfaceUp(interface);
}
void GeoRoutingProtocol::NotifyInterfaceDown(uint32_t interface) {
    if (m_backupProtocol) m_backupProtocol->NotifyInterfaceDown(interface);
}
void GeoRoutingProtocol::NotifyAddAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) {
    if (m_backupProtocol) m_backupProtocol->NotifyAddAddress(interface, address);
}
void GeoRoutingProtocol::NotifyRemoveAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) {
    if (m_backupProtocol) m_backupProtocol->NotifyRemoveAddress(interface, address);
}
void GeoRoutingProtocol::PrintRoutingTable(ns3::Ptr<ns3::OutputStreamWrapper> stream, ns3::Time::Unit unit) const {
    if (m_backupProtocol) m_backupProtocol->PrintRoutingTable(stream, unit);
    *stream->GetStream() << "GeoRouting: Dynamic stateless routing based on GeoRouter." << std::endl;
}

} // namespace myfanet
