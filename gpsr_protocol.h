/*
 * gpsr_protocol.h - Standalone GPSR (Greedy Perimeter Stateless Routing)
 *
 * INDEPENDENT implementation with NO dependency on PGH, LMSS, or clustering.
 * Implements:
 *   1. Greedy forwarding (neighbor closest to destination)
 *   2. Perimeter mode (right-hand rule on Gabriel Graph)
 *   3. Position beaconing (periodic neighbor discovery)
 */

#ifndef MYFANET_GPSR_PROTOCOL_H
#define MYFANET_GPSR_PROTOCOL_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>

namespace myfanet {

struct GpsrNeighborEntry {
    uint32_t nodeId;
    ns3::Ipv4Address address;
    ns3::Vector position;
    ns3::Time lastSeen;
    bool IsValid(ns3::Time now, ns3::Time timeout) const {
        return (now - lastSeen) < timeout;
    }
};

class GpsrRoutingProtocol : public ns3::Ipv4RoutingProtocol {
public:
    static ns3::TypeId GetTypeId();
    GpsrRoutingProtocol();
    ~GpsrRoutingProtocol() override;

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
    static constexpr double BEACON_INTERVAL = 1.0;
    static constexpr double NEIGHBOR_TIMEOUT = 3.0;

    ns3::Ptr<ns3::Ipv4> m_ipv4;
    uint32_t m_myNodeId = 0;
    std::map<uint32_t, GpsrNeighborEntry> m_neighbors;

    uint32_t m_greedySuccess = 0;
    uint32_t m_perimeterSuccess = 0;
    uint32_t m_routeFailures = 0;

    // Core GPSR algorithms
    uint32_t GreedyForward(uint32_t dstNodeId);
    uint32_t PerimeterForward(uint32_t dstNodeId);
    bool IsGabrielNeighbor(uint32_t u, uint32_t v);

    // Beaconing
    void BeaconTimerExpire();
    void UpdateNeighborTable();
    void PurgeExpiredNeighbors();

    // Helpers
    ns3::Ptr<ns3::Ipv4Route> BuildRoute(uint32_t nextHopId, ns3::Ipv4Address dst);
    uint32_t FindNextHop(uint32_t dstNodeId);
    static uint32_t IpToNodeId(ns3::Ipv4Address addr);
    static ns3::Vector GetNodePos(uint32_t nodeId);
    static double Dist(const ns3::Vector &a, const ns3::Vector &b);
    uint32_t GetFanetNodeCount() const;
};

// Helper for installing GPSR on nodes
class GpsrHelper : public ns3::Ipv4RoutingHelper {
public:
    GpsrHelper() {}
    GpsrHelper *Copy() const override { return new GpsrHelper(*this); }
    ns3::Ptr<ns3::Ipv4RoutingProtocol> Create(ns3::Ptr<ns3::Node> node) const override {
        return ns3::CreateObject<GpsrRoutingProtocol>();
    }
};

} // namespace myfanet

#endif // MYFANET_GPSR_PROTOCOL_H
