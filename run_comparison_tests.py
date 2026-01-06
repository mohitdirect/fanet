#!/usr/bin/env python3
"""
Protocol Comparison Test Script
Runs PGH, CR-ED, AODV, OLSR for nodes 2-30 and plots throughput/PDR graphs
"""

import subprocess
import re
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

# Configuration
NS3_DIR = "/home/mohit/ns-allinone-3.43/ns-3.43"
PROTOCOLS = ["pgh", "cr-ed", "aodv", "olsr"]
NODE_RANGE = range(2, 31, 2)  # 2, 4, 6, ..., 30 (steps of 2 for faster testing)
TRAFFIC_MODE = "cluster"  # Random traffic for fairness
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def run_simulation(protocol, num_nodes):
    """Run a single simulation and extract metrics"""
    cmd = [
        "./ns3", "run",
        f"scratch/myfanet/pgh --protocol={protocol} --nNodes={num_nodes} --traffic={TRAFFIC_MODE}"
    ]
    
    try:
        result = subprocess.run(
            cmd,
            cwd=NS3_DIR,
            capture_output=True,
            text=True,
            timeout=300  # 5 minute timeout
        )
        output = result.stdout + result.stderr
        
        # Extract throughput
        throughput_match = re.search(r'Total Throughput.*?:\s*([\d.]+)\s*Mbps', output)
        throughput = float(throughput_match.group(1)) if throughput_match else 0.0
        
        # Extract PDR
        pdr_match = re.search(r'Packet Delivery Ratio.*?:\s*([\d.]+)\s*%', output)
        pdr = float(pdr_match.group(1)) if pdr_match else 0.0
        
        return throughput, pdr
        
    except subprocess.TimeoutExpired:
        print(f"  Timeout for {protocol} with {num_nodes} nodes")
        return 0.0, 0.0
    except Exception as e:
        print(f"  Error: {e}")
        return 0.0, 0.0

def main():
    print("=" * 60)
    print("PROTOCOL COMPARISON TEST")
    print(f"Protocols: {', '.join(PROTOCOLS)}")
    print(f"Nodes: {list(NODE_RANGE)}")
    print("=" * 60)
    
    # Results storage
    results = {p: {"nodes": [], "throughput": [], "pdr": []} for p in PROTOCOLS}
    
    # Run all tests
    total_tests = len(PROTOCOLS) * len(NODE_RANGE)
    test_num = 0
    
    for num_nodes in NODE_RANGE:
        print(f"\n--- Testing with {num_nodes} nodes ---")
        
        for protocol in PROTOCOLS:
            test_num += 1
            print(f"[{test_num}/{total_tests}] {protocol.upper()} with {num_nodes} nodes...", end=" ", flush=True)
            
            throughput, pdr = run_simulation(protocol, num_nodes)
            
            results[protocol]["nodes"].append(num_nodes)
            results[protocol]["throughput"].append(throughput)
            results[protocol]["pdr"].append(pdr)
            
            print(f"Throughput: {throughput:.3f} Mbps, PDR: {pdr:.1f}%")
    
    # Save results to CSV
    csv_file = os.path.join(OUTPUT_DIR, "protocol_comparison_results.csv")
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Nodes", "Protocol", "Throughput_Mbps", "PDR_Percent"])
        for protocol in PROTOCOLS:
            for i, nodes in enumerate(results[protocol]["nodes"]):
                writer.writerow([
                    nodes,
                    protocol,
                    results[protocol]["throughput"][i],
                    results[protocol]["pdr"][i]
                ])
    print(f"\nResults saved to: {csv_file}")
    
    # Generate graphs
    plot_graphs(results)

def plot_graphs(results):
    """Generate comparison graphs"""
    
    # Style configuration
    colors = {'pgh': '#2ecc71', 'cr-ed': '#3498db', 'aodv': '#e74c3c', 'olsr': '#9b59b6'}
    markers = {'pgh': 'o', 'cr-ed': 's', 'aodv': '^', 'olsr': 'd'}
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Throughput graph
    ax1.set_title('Average Throughput vs Number of Nodes', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Number of Nodes', fontsize=12)
    ax1.set_ylabel('Throughput (Mbps)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    
    for protocol in PROTOCOLS:
        ax1.plot(
            results[protocol]["nodes"],
            results[protocol]["throughput"],
            marker=markers[protocol],
            color=colors[protocol],
            label=protocol.upper(),
            linewidth=2,
            markersize=8
        )
    ax1.legend(loc='best', fontsize=10)
    
    # PDR graph
    ax2.set_title('Packet Delivery Ratio vs Number of Nodes', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Number of Nodes', fontsize=12)
    ax2.set_ylabel('PDR (%)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    
    for protocol in PROTOCOLS:
        ax2.plot(
            results[protocol]["nodes"],
            results[protocol]["pdr"],
            marker=markers[protocol],
            color=colors[protocol],
            label=protocol.upper(),
            linewidth=2,
            markersize=8
        )
    ax2.legend(loc='best', fontsize=10)
    
    plt.tight_layout()
    
    # Save graph
    graph_file = os.path.join(OUTPUT_DIR, "protocol_comparison_graph.png")
    plt.savefig(graph_file, dpi=150, bbox_inches='tight')
    print(f"Graphs saved to: {graph_file}")
    
    plt.close()

if __name__ == "__main__":
    main()
