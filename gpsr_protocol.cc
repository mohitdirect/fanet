/*
 * gpsr_protocol.cc - Standalone GPSR Implementation
 *
 * Greedy Perimeter Stateless Routing with NO PGH/LMSS dependency.
 * Uses NS-3 NodeList and MobilityModel for position information.
 */

#include "gpsr_protocol.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-route.h"
#include <algorithm>
#include <cmath>

NS_LOG_COMPONENT_DEFINE("GpsrRouting");

namespace myfanet {

NS_OBJECT_ENSURE_REGISTERED(GpsrRoutingProtocol);

ns3::TypeId GpsrRoutingProtocol::GetTypeId() {
    static ns3::TypeId tid = ns3::TypeId("myfanet::GpsrRoutingProtocol")
        .SetParent<ns3::Ipv4RoutingProtocol>()
        .SetGroupName("Internet")
        .AddConstructor<GpsrRoutingProtocol>();
    return tid;
}

GpsrRoutingProtocol::GpsrRoutingProtocol() : m_ipv4(nullptr) {}
GpsrRoutingProtocol::~GpsrRoutingProtocol() {}

void GpsrRoutingProtocol::SetIpv4(ns3::Ptr<ns3::Ipv4> ipv4) {
    m_ipv4 = ipv4;
    m_myNodeId = m_ipv4->GetObject<ns3::Node>()->GetId();
    ns3::Simulator::Schedule(ns3::Seconds(0.5 + m_myNodeId * 0.01),
                             &GpsrRoutingProtocol::BeaconTimerExpire, this);
}

uint32_t GpsrRoutingProtocol::GetFanetNodeCount() const {
    return ns3::NodeList::GetNNodes() - 1;
}

uint32_t GpsrRoutingProtocol::IpToNodeId(ns3::Ipv4Address addr) {
    if (addr.IsBroadcast()) return UINT32_MAX;
    uint8_t buf[4];
    addr.Serialize(buf);
    if (buf[0] == 10 && buf[1] == 1 && buf[2] == 1) {
        return buf[3] - 1;
    }
    return UINT32_MAX;
}

ns3::Vector GpsrRoutingProtocol::GetNodePos(uint32_t nodeId) {
    ns3::Ptr<ns3::Node> node = ns3::NodeList::GetNode(nodeId);
    ns3::Ptr<ns3::MobilityModel> mm = node->GetObject<ns3::MobilityModel>();
    return mm ? mm->GetPosition() : ns3::Vector(0, 0, 0);
}

double GpsrRoutingProtocol::Dist(const ns3::Vector &a, const ns3::Vector &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) +
                     std::pow(a.y - b.y, 2) +
                     std::pow(a.z - b.z, 2));
}

// ==================== BEACONING ====================

void GpsrRoutingProtocol::BeaconTimerExpire() {
    UpdateNeighborTable();
    PurgeExpiredNeighbors();
    ns3::Simulator::Schedule(ns3::Seconds(BEACON_INTERVAL),
                             &GpsrRoutingProtocol::BeaconTimerExpire, this);
}

void GpsrRoutingProtocol::UpdateNeighborTable() {
    ns3::Vector myPos = GetNodePos(m_myNodeId);
    ns3::Time now = ns3::Simulator::Now();
    uint32_t numNodes = GetFanetNodeCount();

    for (uint32_t i = 0; i < numNodes; i++) {
        if (i == m_myNodeId) continue;
        ns3::Vector pos = GetNodePos(i);
        if (Dist(myPos, pos) < COMM_RANGE) {
            GpsrNeighborEntry entry;
            entry.nodeId = i;
            entry.position = pos;
            entry.lastSeen = now;
            entry.address = ns3::Ipv4Address((10 << 24) | (1 << 16) | (1 << 8) | (i + 1));
            m_neighbors[i] = entry;
        }
    }
}

void GpsrRoutingProtocol::PurgeExpiredNeighbors() {
    ns3::Time now = ns3::Simulator::Now();
    ns3::Time timeout = ns3::Seconds(NEIGHBOR_TIMEOUT);
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ) {
        if (!it->second.IsValid(now, timeout)) {
            it = m_neighbors.erase(it);
        } else {
            ++it;
        }
    }
}

// ==================== GREEDY FORWARDING ====================

uint32_t GpsrRoutingProtocol::GreedyForward(uint32_t dstNodeId) {
    ns3::Vector myPos = GetNodePos(m_myNodeId);
    ns3::Vector dstPos = GetNodePos(dstNodeId);
    double myDistToDst = Dist(myPos, dstPos);

    uint32_t bestNeighbor = UINT32_MAX;
    double bestDist = myDistToDst;

    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        double d = Dist(it->second.position, dstPos);
        if (d < bestDist) {
            bestDist = d;
            bestNeighbor = it->first;
        }
    }

    if (bestNeighbor != UINT32_MAX) {
        m_greedySuccess++;
    }
    return bestNeighbor;
}

// ==================== GABRIEL GRAPH ====================

bool GpsrRoutingProtocol::IsGabrielNeighbor(uint32_t u, uint32_t v) {
    ns3::Vector posU = GetNodePos(u);
    ns3::Vector posV = GetNodePos(v);

    ns3::Vector mid;
    mid.x = (posU.x + posV.x) / 2.0;
    mid.y = (posU.y + posV.y) / 2.0;
    mid.z = (posU.z + posV.z) / 2.0;
    double radius = Dist(posU, posV) / 2.0;

    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        uint32_t wId = it->first;
        if (wId == u || wId == v) continue;
        if (Dist(it->second.position, mid) < radius) {
            return false;
        }
    }
    return true;
}

// ==================== PERIMETER FORWARDING ====================

uint32_t GpsrRoutingProtocol::PerimeterForward(uint32_t dstNodeId) {
    ns3::Vector myPos = GetNodePos(m_myNodeId);
    ns3::Vector dstPos = GetNodePos(dstNodeId);

    double angleToDst = std::atan2(dstPos.y - myPos.y, dstPos.x - myPos.x);

    std::vector<std::pair<uint32_t, double>> gabrielNeighbors;
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        if (IsGabrielNeighbor(m_myNodeId, it->first)) {
            double angle = std::atan2(
                it->second.position.y - myPos.y,
                it->second.position.x - myPos.x);
            gabrielNeighbors.push_back(std::make_pair(it->first, angle));
        }
    }

    if (gabrielNeighbors.empty()) return UINT32_MAX;

    double refAngle = angleToDst + M_PI;
    double bestDiff = 2 * M_PI + 1;
    uint32_t bestNeighbor = UINT32_MAX;

    for (size_t i = 0; i < gabrielNeighbors.size(); i++) {
        double diff = gabrielNeighbors[i].second - refAngle;
        while (diff < 0) diff += 2 * M_PI;
        while (diff >= 2 * M_PI) diff -= 2 * M_PI;

        if (diff < bestDiff) {
            bestDiff = diff;
            bestNeighbor = gabrielNeighbors[i].first;
        }
    }

    if (bestNeighbor != UINT32_MAX) {
        m_perimeterSuccess++;
    }
    return bestNeighbor;
}

// ==================== COMBINED NEXT HOP ====================

uint32_t GpsrRoutingProtocol::FindNextHop(uint32_t dstNodeId) {
    if (m_neighbors.count(dstNodeId)) return dstNodeId;

    uint32_t hop = GreedyForward(dstNodeId);
    if (hop != UINT32_MAX) return hop;

    hop = PerimeterForward(dstNodeId);
    if (hop != UINT32_MAX) return hop;

    m_routeFailures++;
    return UINT32_MAX;
}

// ==================== ROUTE BUILDING ====================

ns3::Ptr<ns3::Ipv4Route> GpsrRoutingProtocol::BuildRoute(uint32_t nextHopId, ns3::Ipv4Address dst) {
    ns3::Ipv4Address nextHopIp((10 << 24) | (1 << 16) | (1 << 8) | (nextHopId + 1));
    ns3::Ptr<ns3::Ipv4Route> route = ns3::Create<ns3::Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHopIp);
    route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    return route;
}

// ==================== ROUTE OUTPUT ====================

ns3::Ptr<ns3::Ipv4Route> GpsrRoutingProtocol::RouteOutput(
    ns3::Ptr<ns3::Packet> p,
    const ns3::Ipv4Header &header,
    ns3::Ptr<ns3::NetDevice> oif,
    ns3::Socket::SocketErrno &sockerr) {

    uint32_t dstId = IpToNodeId(header.GetDestination());
    if (dstId == UINT32_MAX) {
        sockerr = ns3::Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }

    uint32_t nextHop = FindNextHop(dstId);
    if (nextHop != UINT32_MAX) {
        return BuildRoute(nextHop, header.GetDestination());
    }

    sockerr = ns3::Socket::ERROR_NOROUTETOHOST;
    return nullptr;
}

// ==================== ROUTE INPUT ====================

bool GpsrRoutingProtocol::RouteInput(
    ns3::Ptr<const ns3::Packet> p,
    const ns3::Ipv4Header &header,
    ns3::Ptr<const ns3::NetDevice> idev,
    const UnicastForwardCallback &ucb,
    const MulticastForwardCallback &mcb,
    const LocalDeliverCallback &lcb,
    const ErrorCallback &ecb) {

    if (m_ipv4->IsDestinationAddress(header.GetDestination(), idev->GetIfIndex())) {
        lcb(p, header, idev->GetIfIndex());
        return true;
    }

    if (header.GetDestination().IsBroadcast() || header.GetDestination().IsMulticast()) {
        return false;
    }

    uint32_t dstId = IpToNodeId(header.GetDestination());
    if (dstId == UINT32_MAX) return false;

    uint32_t nextHop = FindNextHop(dstId);
    if (nextHop != UINT32_MAX) {
        ucb(BuildRoute(nextHop, header.GetDestination()), p, header);
        return true;
    }

    return false;
}

// ==================== NOTIFICATIONS ====================

void GpsrRoutingProtocol::NotifyInterfaceUp(uint32_t) {}
void GpsrRoutingProtocol::NotifyInterfaceDown(uint32_t) {}
void GpsrRoutingProtocol::NotifyAddAddress(uint32_t, ns3::Ipv4InterfaceAddress) {}
void GpsrRoutingProtocol::NotifyRemoveAddress(uint32_t, ns3::Ipv4InterfaceAddress) {}

void GpsrRoutingProtocol::PrintRoutingTable(
    ns3::Ptr<ns3::OutputStreamWrapper> stream, ns3::Time::Unit) const {
    *stream->GetStream() << "GPSR: Stateless geographic routing" << std::endl;
    *stream->GetStream() << "  Neighbors: " << m_neighbors.size() << std::endl;
    *stream->GetStream() << "  Greedy: " << m_greedySuccess
                         << ", Perimeter: " << m_perimeterSuccess
                         << ", Failures: " << m_routeFailures << std::endl;
}

} // namespace myfanet
