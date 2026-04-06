/*
 * cbrp_protocol.h - Standalone CBRP (Cluster-Based Routing Protocol)
 *
 * INDEPENDENT implementation with NO dependency on PGH, LMSS, or GeoRouter.
 * Implements:
 *   1. Lowest-ID cluster head election
 *   2. HELLO-based neighbor discovery with cluster info
 *   3. Intra-cluster routing via CH
 *   4. Inter-cluster routing via gateways
 */

#ifndef MYFANET_CBRP_PROTOCOL_H
#define MYFANET_CBRP_PROTOCOL_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <set>

namespace myfanet {

enum class CbrpRole {
    UNDECIDED,
    CLUSTER_HEAD,
    MEMBER,
    GATEWAY  // Member with neighbors in a different cluster
};

struct CbrpNeighborEntry {
    uint32_t nodeId;
    ns3::Ipv4Address address;
    ns3::Vector position;
    CbrpRole role;
    uint32_t clusterId;  // Which CH this neighbor belongs to
    ns3::Time lastSeen;
    bool IsValid(ns3::Time now, ns3::Time timeout) const {
        return (now - lastSeen) < timeout;
    }
};

class CbrpRoutingProtocol : public ns3::Ipv4RoutingProtocol {
public:
    static ns3::TypeId GetTypeId();
    CbrpRoutingProtocol();
    ~CbrpRoutingProtocol() override;

    // Ipv4RoutingProtocol interface
    ns3::Ptr<ns3::Ipv4Route> RouteOutput(ns3::Ptr<ns3::Packet> p,
                                          const ns3::Ipv4Header &header,
                                          ns3::Ptr<ns3::NetDevice> oif,
                                          ns3::Socket::SocketErrno &sockerr) override;

    bool RouteInput(ns3::Ptr<const ns3::Packet> p,
                    const ns3::Ipv4Header &header,
                    ns3::Ptr<const ns3::NetDevice> idev,
                    const UnicastForwardCallback &ucb,
                    const MulticastForwardCallback &mcb,
                    const LocalDeliverCallback &lcb,
                    const ErrorCallback &ecb) override;

    void SetIpv4(ns3::Ptr<ns3::Ipv4> ipv4) override;
    void NotifyInterfaceUp(uint32_t interface) override;
    void NotifyInterfaceDown(uint32_t interface) override;
    void NotifyAddAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) override;
    void NotifyRemoveAddress(uint32_t interface, ns3::Ipv4InterfaceAddress address) override;
    void PrintRoutingTable(ns3::Ptr<ns3::OutputStreamWrapper> stream,
                           ns3::Time::Unit unit) const override;

private:
    static constexpr double COMM_RANGE = 600.0;
    static constexpr double HELLO_INTERVAL = 1.0;
    static constexpr double NEIGHBOR_TIMEOUT = 3.0;
    static constexpr double CLUSTER_UPDATE_INTERVAL = 2.0;

    ns3::Ptr<ns3::Ipv4> m_ipv4;
    uint32_t m_myNodeId = 0;
    CbrpRole m_myRole = CbrpRole::UNDECIDED;
    uint32_t m_myClusterId = UINT32_MAX;

    std::map<uint32_t, CbrpNeighborEntry> m_neighbors;
    // Global knowledge of node→cluster mapping (learned from HELLOs)
    std::map<uint32_t, uint32_t> m_nodeClusterMap;

    uint32_t m_intraRoutes = 0;
    uint32_t m_interRoutes = 0;
    uint32_t m_routeFailures = 0;

    // HELLO / neighbor management
    void HelloTimerExpire();
    void UpdateNeighborTable();
    void PurgeExpiredNeighbors();

    // Lowest-ID clustering
    void ClusterTimerExpire();
    void LowestIdClustering();
    void DetectGateways();

    // Routing decisions
    uint32_t FindNextHop(uint32_t dstNodeId);
    uint32_t IntraClusterRoute(uint32_t dstNodeId);
    uint32_t InterClusterRoute(uint32_t dstNodeId);
    uint32_t FindGatewayToCluster(uint32_t targetClusterId);

    // Helpers
    ns3::Ptr<ns3::Ipv4Route> BuildRoute(uint32_t nextHopId, ns3::Ipv4Address dst);
    static uint32_t IpToNodeId(ns3::Ipv4Address addr);
    static ns3::Vector GetNodePos(uint32_t nodeId);
    static double Dist(const ns3::Vector &a, const ns3::Vector &b);
    uint32_t GetFanetNodeCount() const;
};

// Helper for installing CBRP on nodes
class CbrpHelper : public ns3::Ipv4RoutingHelper {
public:
    CbrpHelper() {}
    CbrpHelper *Copy() const override { return new CbrpHelper(*this); }
    ns3::Ptr<ns3::Ipv4RoutingProtocol> Create(ns3::Ptr<ns3::Node> node) const override {
        return ns3::CreateObject<CbrpRoutingProtocol>();
    }
};

} // namespace myfanet

#endif // MYFANET_CBRP_PROTOCOL_H
