# Anti-Jamming Resilient MANET Protocol for Military Aircraft Operations

## Project Context

Modern military operations rely heavily on Mobile Ad-hoc Networks (MANETs) formed by tactical aircraft for real-time coordination, situational awareness sharing, and mission-critical communications. These networks operate in highly contested electromagnetic environments where adversaries deploy jamming systems to disrupt communications and degrade operational effectiveness.

### Operational Scenario

```
   ✈️ Fighter Squadron          🛡️ Adversary Jammer
      (MANET Nodes)                (RF Interference)
           │                             │
    ┌──────┴──────┐                      │
    │  Mission    │◄──── Jamming ────────┘
    │  Critical   │
    │  Data Link  │
    └─────────────┘
```

**Challenges:**
- High-speed aircraft mobility (Mach 0.8-2.0)
- 3D maneuvering with altitude changes
- Hostile RF environment with active jamming
- Need for rapid network reconfiguration
- Mission continuity requirements

---

## Proposed Solution: PGH-LMSS Protocol

### Core Innovation: 6-Component Link Mobility Stability Score (LMSS)

A novel metric designed specifically for military aircraft MANETs:

| Component | Military Relevance |
|-----------|-------------------|
| **Φ1: Link Duration** | Identifies stable wingman pairs during formation flying |
| **Φ2: Distance Stability** | Accounts for tactical spacing in combat formations |
| **Φ3: Relative Velocity** | Handles speed differentials in mixed aircraft types |
| **Φ4: Heading Alignment** | Exploits aircraft flying same heading vector |
| **Φ5: Altitude Stability** | 3D layered formations common in air operations |
| **Φ6: Flight Predictability** | Leverages predictable flight paths for routing |

### Anti-Jamming Capability: Link Spectrum Stability Score (LSSS)

Monitors spectrum quality and detects jamming through:
- **SINR-based detection**: Real-time signal quality monitoring
- **PER-based detection**: Packet error rate analysis
- **Cooperative detection**: Multiple aircraft confirming jammer presence
- **Proactive handoff**: Frequency switching before communication loss

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Cross-Layer Controller                     │
├─────────────┬─────────────┬──────────────┬─────────────────┤
│  PHY Layer  │  MAC Layer  │  Network     │  Application    │
│  - SINR     │  - Collision│  - Route     │  - Throughput   │
│  - PER      │  - Backoff  │  - Stability │  - Delay        │
└──────┬──────┴──────┬──────┴──────┬───────┴────────┬────────┘
       │             │             │                │
       └─────────────┴─────────────┴────────────────┘
                           │
                    ┌──────▼──────┐
                    │  Adaptive   │
                    │  LMSS/LSSS  │
                    │  Weights    │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
   ┌─────────┐       ┌─────────┐       ┌─────────┐
   │ Cluster │       │ Super   │       │ Handoff │
   │Formation│       │ Cluster │       │ Decision│
   └─────────┘       └─────────┘       └─────────┘
```

---

## Hierarchical Clustering for Tactical Operations

### Two-Tier Structure

**Local Clusters (Flight Level)**
- 5-10 aircraft per cluster
- Cluster Head elected by highest LMSS score
- Intra-cluster: Direct link or single-hop via CH

**Super-Clusters (Squadron Level)**
- Groups multiple flights for coordination
- Super-Cluster Head for inter-squadron routing
- Backbone routing via OLSR

### Military Benefit
- Mirrors actual military command structure (Flight → Squadron → Wing)
- Reduces routing overhead under jamming
- Enables decentralized decision-making

---

## Simulation Results

### Test Configuration
- **Nodes**: 6-100 aircraft
- **Area**: 1000m × 1000m × 100m (altitude variation)
- **Mobility**: Gauss-Markov (represents aircraft movement)
- **Jammer**: Approaches network at t=17.5s

### Performance Under Jamming

| Metric | PGH-LMSS | AODV | OLSR | Improvement |
|--------|----------|------|------|-------------|
| **Throughput (30 nodes)** | 1.27 Mbps | 0.68 Mbps | 0.93 Mbps | +37% vs OLSR |
| **Jammer Detection** | 0.8-1.2s | N/A | N/A | Proactive |
| **Recovery Time** | 3.5s | >10s | >10s | 3× faster |
| **PDR (50 nodes)** | 26.1% | 12.6% | 27.4% | Comparable |

### Key Finding
> PGH-LMSS achieves **faster jammer detection** and **quicker recovery** while maintaining throughput comparable to or better than standard protocols in contested environments.

---

## Protocol Comparison

| Feature | PGH-LMSS | CR-ED | AODV | OLSR |
|---------|----------|-------|------|------|
| 3D Mobility Awareness | ✓ | ✗ | ✗ | ✗ |
| Anti-Jamming Detection | ✓ | ✓ | ✗ | ✗ |
| Proactive Handoff | ✓ | ✓ | ✗ | ✗ |
| Hierarchical Clustering | ✓ | ✗ | ✗ | ✗ |
| Cross-Layer Adaptation | ✓ | Partial | ✗ | ✗ |
| Flight Formation Aware | ✓ | ✗ | ✗ | ✗ |

---

## Implementation Details

**Simulation Platform**: NS-3.43  
**Language**: C++ (~5000+ lines)  
**Modules**: 12 source files

### Core Modules
- `lmss.cc` - 6-Component LMSS computation
- `handoff.cc` - Cluster formation and spectrum handoff
- `cross_layer_controller.cc` - Cross-layer information fusion
- `sinr_monitor.cc` - Real-time spectrum monitoring
- `load_balancer.cc` - Traffic distribution for CH congestion avoidance

---

## Contributions

1. **Novel 6-Component LMSS** formulation for 3D aircraft mobility
2. **LSSS-aware clustering** that avoids jammer-affected nodes
3. **Cross-layer adaptive weights** responding to jamming conditions
4. **Hierarchical super-clustering** matching military command structure
5. **Comprehensive comparison** against AODV, OLSR, and CR-ED

---

## Future Work

- Integration with Link-16/MADL tactical data link specifications
- Multi-jammer scenario handling
- Frequency hopping coordination
- Scalability optimization for 100+ aircraft (large-scale operations)

---

## Conclusion

The PGH-LMSS protocol demonstrates that anti-jamming resilient MANETs for military aircraft operations are achievable through:

1. **Mobility-aware link metrics** tuned for 3D aircraft dynamics
2. **Proactive spectrum sensing** for early jammer detection
3. **Hierarchical organization** matching tactical command structures
4. **Cross-layer adaptation** for dynamic weight optimization

This approach enables military aircraft to **maintain mission-critical communications** even in contested electromagnetic environments.

---

*Project developed for Major's Project submission - February 2026*
