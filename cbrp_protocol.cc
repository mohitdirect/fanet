/*
 * cbrp_protocol.cc - Standalone CBRP Implementation
 *
 * Cluster-Based Routing Protocol with lowest-ID CH election.
 * NO dependency on PGH, LMSS, GeoRouter, or any other module.
 * Uses NS-3 NodeList and MobilityModel for neighbor discovery.
 */

#include "cbrp_protocol.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-route.h"
#include <algorithm>
#include <cmath>

NS_LOG_COMPONENT_DEFINE("CbrpRouting");

namespace myfanet {

NS_OBJECT_ENSURE_REGISTERED(CbrpRoutingProtocol);

ns3::TypeId CbrpRoutingProtocol::GetTypeId() {
    static ns3::TypeId tid = ns3::TypeId("myfanet::CbrpRoutingProtocol")
        .SetParent<ns3::Ipv4RoutingProtocol>()
        .SetGroupName("Internet")
        .AddConstructor<CbrpRoutingProtocol>();
    return tid;
}

CbrpRoutingProtocol::CbrpRoutingProtocol() : m_ipv4(nullptr) {}
CbrpRoutingProtocol::~CbrpRoutingProtocol() {}

void CbrpRoutingProtocol::SetIpv4(ns3::Ptr<ns3::Ipv4> ipv4) {
    m_ipv4 = ipv4;
    m_myNodeId = m_ipv4->GetObject<ns3::Node>()->GetId();
    ns3::Simulator::Schedule(ns3::Seconds(0.5 + m_myNodeId * 0.01),
                             &CbrpRoutingProtocol::HelloTimerExpire, this);
    ns3::Simulator::Schedule(ns3::Seconds(1.5 + m_myNodeId * 0.01),
                             &CbrpRoutingProtocol::ClusterTimerExpire, this);
}

uint32_t CbrpRoutingProtocol::GetFanetNodeCount() const {
    return ns3::NodeList::GetNNodes() - 1;
}

uint32_t CbrpRoutingProtocol::IpToNodeId(ns3::Ipv4Address addr) {
    if (addr.IsBroadcast()) return UINT32_MAX;
    uint8_t buf[4];
    addr.Serialize(buf);
    if (buf[0] == 10 && buf[1] == 1 && buf[2] == 1) {
        return buf[3] - 1;
    }
    return UINT32_MAX;
}

ns3::Vector CbrpRoutingProtocol::GetNodePos(uint32_t nodeId) {
    ns3::Ptr<ns3::Node> node = ns3::NodeList::GetNode(nodeId);
    ns3::Ptr<ns3::MobilityModel> mm = node->GetObject<ns3::MobilityModel>();
    return mm ? mm->GetPosition() : ns3::Vector(0, 0, 0);
}

double CbrpRoutingProtocol::Dist(const ns3::Vector &a, const ns3::Vector &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) +
                     std::pow(a.y - b.y, 2) +
                     std::pow(a.z - b.z, 2));
}

// ==================== HELLO / NEIGHBOR MANAGEMENT ====================

void CbrpRoutingProtocol::HelloTimerExpire() {
    UpdateNeighborTable();
    PurgeExpiredNeighbors();
    ns3::Simulator::Schedule(ns3::Seconds(HELLO_INTERVAL),
                             &CbrpRoutingProtocol::HelloTimerExpire, this);
}

void CbrpRoutingProtocol::UpdateNeighborTable() {
    ns3::Vector myPos = GetNodePos(m_myNodeId);
    ns3::Time now = ns3::Simulator::Now();
    uint32_t numNodes = GetFanetNodeCount();

    for (uint32_t i = 0; i < numNodes; i++) {
        if (i == m_myNodeId) continue;
        ns3::Vector pos = GetNodePos(i);
        if (Dist(myPos, pos) < COMM_RANGE) {
            CbrpNeighborEntry entry;
            entry.nodeId = i;
            entry.position = pos;
            entry.lastSeen = now;
            entry.address = ns3::Ipv4Address((10 << 24) | (1 << 16) | (1 << 8) | (i + 1));
            if (m_neighbors.count(i)) {
                entry.role = m_neighbors[i].role;
                entry.clusterId = m_neighbors[i].clusterId;
            } else {
                entry.role = CbrpRole::UNDECIDED;
                entry.clusterId = UINT32_MAX;
            }
            m_neighbors[i] = entry;
        }
    }
}

void CbrpRoutingProtocol::PurgeExpiredNeighbors() {
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

// ==================== LOWEST-ID CLUSTERING ====================

void CbrpRoutingProtocol::ClusterTimerExpire() {
    LowestIdClustering();
    DetectGateways();

    // Share state with neighbors (simulated HELLO exchange)
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        ns3::Ptr<ns3::Node> nNode = ns3::NodeList::GetNode(it->first);
        ns3::Ptr<ns3::Ipv4> nIpv4 = nNode->GetObject<ns3::Ipv4>();
        if (nIpv4) {
            ns3::Ptr<ns3::Ipv4RoutingProtocol> rp = nIpv4->GetRoutingProtocol();
            CbrpRoutingProtocol* cbrp = dynamic_cast<CbrpRoutingProtocol*>(ns3::PeekPointer(rp));
            if (cbrp) {
                it->second.role = cbrp->m_myRole;
                it->second.clusterId = cbrp->m_myClusterId;
                m_nodeClusterMap[it->first] = cbrp->m_myClusterId;
            }
        }
    }
    m_nodeClusterMap[m_myNodeId] = m_myClusterId;

    ns3::Simulator::Schedule(ns3::Seconds(CLUSTER_UPDATE_INTERVAL),
                             &CbrpRoutingProtocol::ClusterTimerExpire, this);
}

void CbrpRoutingProtocol::LowestIdClustering() {
    if (m_neighbors.empty()) {
        m_myRole = CbrpRole::CLUSTER_HEAD;
        m_myClusterId = m_myNodeId;
        return;
    }

    uint32_t lowestId = m_myNodeId;
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        if (it->first < lowestId) {
            lowestId = it->first;
        }
    }

    if (lowestId == m_myNodeId) {
        m_myRole = CbrpRole::CLUSTER_HEAD;
        m_myClusterId = m_myNodeId;
    } else {
        uint32_t bestCh = UINT32_MAX;
        double bestDist = 1e9;
        ns3::Vector myPos = GetNodePos(m_myNodeId);

        for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
            if (it->second.role == CbrpRole::CLUSTER_HEAD) {
                double d = Dist(myPos, it->second.position);
                if (it->first < bestCh || (it->first == bestCh && d < bestDist)) {
                    bestCh = it->first;
                    bestDist = d;
                }
            }
        }

        if (bestCh != UINT32_MAX) {
            m_myRole = CbrpRole::MEMBER;
            m_myClusterId = bestCh;
        } else {
            m_myRole = CbrpRole::MEMBER;
            m_myClusterId = lowestId;
        }
    }
}

void CbrpRoutingProtocol::DetectGateways() {
    if (m_myRole != CbrpRole::MEMBER) return;

    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        if (it->second.clusterId != UINT32_MAX && it->second.clusterId != m_myClusterId) {
            m_myRole = CbrpRole::GATEWAY;
            return;
        }
    }
}

// ==================== ROUTING ====================

uint32_t CbrpRoutingProtocol::FindNextHop(uint32_t dstNodeId) {
    // 1. Direct neighbor
    if (m_neighbors.count(dstNodeId)) return dstNodeId;

    // 2. Destination cluster
    uint32_t dstCluster = UINT32_MAX;
    if (m_nodeClusterMap.count(dstNodeId)) {
        dstCluster = m_nodeClusterMap[dstNodeId];
    }

    // 3. Same cluster → intra-cluster routing
    if (dstCluster == m_myClusterId && dstCluster != UINT32_MAX) {
        uint32_t hop = IntraClusterRoute(dstNodeId);
        if (hop != UINT32_MAX) { m_intraRoutes++; return hop; }
    }

    // 4. Different cluster → inter-cluster routing
    if (dstCluster != UINT32_MAX && dstCluster != m_myClusterId) {
        uint32_t hop = InterClusterRoute(dstNodeId);
        if (hop != UINT32_MAX) { m_interRoutes++; return hop; }
    }

    // 5. Unknown dest → forward to CH
    if (m_myRole == CbrpRole::MEMBER || m_myRole == CbrpRole::GATEWAY) {
        if (m_neighbors.count(m_myClusterId)) { m_interRoutes++; return m_myClusterId; }
    }

    // 6. CH fallback: geographic closest neighbor to destination
    if (m_myRole == CbrpRole::CLUSTER_HEAD) {
        ns3::Vector dstPos = GetNodePos(dstNodeId);
        double bestDist = 1e9;
        uint32_t bestNeighbor = UINT32_MAX;

        // Prefer gateways and CHs
        for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
            if (it->second.role == CbrpRole::GATEWAY || it->second.role == CbrpRole::CLUSTER_HEAD) {
                double d = Dist(it->second.position, dstPos);
                if (d < bestDist) { bestDist = d; bestNeighbor = it->first; }
            }
        }

        // Any neighbor if no gateway/CH
        if (bestNeighbor == UINT32_MAX) {
            for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
                double d = Dist(it->second.position, dstPos);
                if (d < bestDist) { bestDist = d; bestNeighbor = it->first; }
            }
        }

        if (bestNeighbor != UINT32_MAX) { m_interRoutes++; return bestNeighbor; }
    }

    m_routeFailures++;
    return UINT32_MAX;
}

uint32_t CbrpRoutingProtocol::IntraClusterRoute(uint32_t dstNodeId) {
    if (m_myRole == CbrpRole::CLUSTER_HEAD) {
        if (m_neighbors.count(dstNodeId)) return dstNodeId;

        ns3::Vector dstPos = GetNodePos(dstNodeId);
        double bestDist = 1e9;
        uint32_t bestMember = UINT32_MAX;
        for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
            if (it->second.clusterId == m_myClusterId) {
                double d = Dist(it->second.position, dstPos);
                if (d < bestDist) { bestDist = d; bestMember = it->first; }
            }
        }
        return bestMember;
    }

    // Member → forward to CH
    if (m_neighbors.count(m_myClusterId)) return m_myClusterId;
    return UINT32_MAX;
}

uint32_t CbrpRoutingProtocol::InterClusterRoute(uint32_t dstNodeId) {
    uint32_t dstCluster = UINT32_MAX;
    if (m_nodeClusterMap.count(dstNodeId)) {
        dstCluster = m_nodeClusterMap[dstNodeId];
    }

    if (m_myRole == CbrpRole::MEMBER || m_myRole == CbrpRole::GATEWAY) {
        // Gateway connecting to target cluster → forward directly
        if (m_myRole == CbrpRole::GATEWAY && dstCluster != UINT32_MAX) {
            for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
                if (it->second.clusterId == dstCluster) return it->first;
            }
        }
        // Otherwise → forward to CH
        if (m_neighbors.count(m_myClusterId)) return m_myClusterId;
        return UINT32_MAX;
    }

    // CH: find gateway to target cluster
    if (dstCluster != UINT32_MAX) {
        uint32_t gw = FindGatewayToCluster(dstCluster);
        if (gw != UINT32_MAX) return gw;
    }

    // CH fallback: closest gateway/CH toward destination
    ns3::Vector dstPos = GetNodePos(dstNodeId);
    double bestDist = 1e9;
    uint32_t bestHop = UINT32_MAX;

    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        if (it->second.role == CbrpRole::GATEWAY || it->second.role == CbrpRole::CLUSTER_HEAD) {
            double d = Dist(it->second.position, dstPos);
            if (d < bestDist) { bestDist = d; bestHop = it->first; }
        }
    }
    return bestHop;
}

uint32_t CbrpRoutingProtocol::FindGatewayToCluster(uint32_t targetClusterId) {
    // Check if target CH is my direct neighbor
    if (m_neighbors.count(targetClusterId)) return targetClusterId;

    // Find a gateway that connects to the target cluster
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ++it) {
        if (it->second.role == CbrpRole::GATEWAY && it->second.clusterId == m_myClusterId) {
            // Check if any of my neighbors are in the target cluster
            for (auto jt = m_neighbors.begin(); jt != m_neighbors.end(); ++jt) {
                if (jt->second.clusterId == targetClusterId) {
                    return it->first; // Route via this gateway
                }
            }
        }
    }
    return UINT32_MAX;
}

// ==================== ROUTE BUILDING ====================

ns3::Ptr<ns3::Ipv4Route> CbrpRoutingProtocol::BuildRoute(uint32_t nextHopId, ns3::Ipv4Address dst) {
    ns3::Ipv4Address nextHopIp((10 << 24) | (1 << 16) | (1 << 8) | (nextHopId + 1));
    ns3::Ptr<ns3::Ipv4Route> route = ns3::Create<ns3::Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHopIp);
    route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    return route;
}

// ==================== ROUTE OUTPUT ====================

ns3::Ptr<ns3::Ipv4Route> CbrpRoutingProtocol::RouteOutput(
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

bool CbrpRoutingProtocol::RouteInput(
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

void CbrpRoutingProtocol::NotifyInterfaceUp(uint32_t) {}
void CbrpRoutingProtocol::NotifyInterfaceDown(uint32_t) {}
void CbrpRoutingProtocol::NotifyAddAddress(uint32_t, ns3::Ipv4InterfaceAddress) {}
void CbrpRoutingProtocol::NotifyRemoveAddress(uint32_t, ns3::Ipv4InterfaceAddress) {}

void CbrpRoutingProtocol::PrintRoutingTable(
    ns3::Ptr<ns3::OutputStreamWrapper> stream, ns3::Time::Unit) const {
    *stream->GetStream() << "CBRP: Cluster-Based Routing (Lowest-ID)" << std::endl;
    *stream->GetStream() << "  Role: "
        << (m_myRole == CbrpRole::CLUSTER_HEAD ? "CH" :
            m_myRole == CbrpRole::GATEWAY ? "GW" :
            m_myRole == CbrpRole::MEMBER ? "MBR" : "?")
        << ", Cluster: " << m_myClusterId << std::endl;
    *stream->GetStream() << "  Neighbors: " << m_neighbors.size() << std::endl;
    *stream->GetStream() << "  Intra: " << m_intraRoutes
                         << ", Inter: " << m_interRoutes
                         << ", Failures: " << m_routeFailures << std::endl;
}

} // namespace myfanet
