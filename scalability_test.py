#!/usr/bin/env python3
"""
Comprehensive Protocol Scalability Test
Tests PGH, AODV, OLSR from 6 to 100 nodes with graphs for:
- Throughput
- PDR (Packet Delivery Ratio)
- End-to-End Delay
"""

import subprocess
import re
import os
import csv
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, List
from dataclasses import dataclass
import time

# Configuration
NS3_DIR = "/home/mohit/ns-allinone-3.43/ns-3.43"
PROTOCOLS = ["pgh", "cr-ed", "aodv", "olsr"]
NODE_COUNTS = [6, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100]
SIM_TIME = 45  # Reduced for faster testing
RUNS_PER_CONFIG = 3  # Multiple runs for averaging
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

@dataclass
class SimResult:
    throughput: float = 0.0
    pdr: float = 0.0
    delay: float = 0.0
    detection_time: float = -1.0

def run_simulation(protocol: str, num_nodes: int, run: int) -> SimResult:
    """Run a single simulation and extract metrics"""
    cmd = [
        "./ns3", "run",
        f"scratch/myfanet/pgh --protocol={protocol} --nNodes={num_nodes} --simTime={SIM_TIME} --run={run} --seed={run*7+3}"
    ]
    
    result = SimResult()
    
    try:
        proc = subprocess.run(
            cmd,
            cwd=NS3_DIR,
            capture_output=True,
            text=True,
            timeout=600  # 10 minute timeout for larger networks
        )
        output = proc.stdout + proc.stderr
        
        # Extract throughput
        match = re.search(r'Total Throughput.*?:\s*([\d.]+)\s*Mbps', output)
        if match:
            result.throughput = float(match.group(1))
        
        # Extract PDR
        match = re.search(r'Packet Delivery Ratio.*?:\s*([\d.]+)\s*%', output)
        if match:
            result.pdr = float(match.group(1))
        
        # Extract average delay
        match = re.search(r'Avg End-to-End Delay.*?:\s*([\d.]+)\s*s', output)
        if match:
            result.delay = float(match.group(1))
        
        # Extract detection time (PGH only)
        match = re.search(r'Jammer Detection Time:\s*([\d.]+)\s*s', output)
        if match:
            result.detection_time = float(match.group(1))
        
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: {protocol} with {num_nodes} nodes")
    except Exception as e:
        print(f"  ERROR: {e}")
    
    return result

def main():
    print("=" * 70)
    print("  FANET PROTOCOL SCALABILITY COMPARISON TEST")
    print("=" * 70)
    print(f"  Protocols: {', '.join([p.upper() for p in PROTOCOLS])}")
    print(f"  Node counts: {NODE_COUNTS}")
    print(f"  Runs per config: {RUNS_PER_CONFIG}")
    print(f"  Simulation time: {SIM_TIME}s")
    print("=" * 70)
    
    # Results storage: protocol -> nodes -> list of results
    all_results: Dict[str, Dict[int, List[SimResult]]] = {p: {} for p in PROTOCOLS}
    
    total_tests = len(PROTOCOLS) * len(NODE_COUNTS) * RUNS_PER_CONFIG
    test_num = 0
    start_time = time.time()
    
    for num_nodes in NODE_COUNTS:
        print(f"\n{'='*50}")
        print(f"  Testing with {num_nodes} nodes")
        print(f"{'='*50}")
        
        for protocol in PROTOCOLS:
            runs = []
            
            for run in range(1, RUNS_PER_CONFIG + 1):
                test_num += 1
                elapsed = time.time() - start_time
                eta = (elapsed / test_num) * (total_tests - test_num) / 60 if test_num > 0 else 0
                
                print(f"[{test_num:3d}/{total_tests}] {protocol.upper():5s} n={num_nodes:3d} run={run} (ETA: {eta:.1f}m)...", 
                      end=" ", flush=True)
                
                result = run_simulation(protocol, num_nodes, run)
                runs.append(result)
                
                print(f"T={result.throughput:.2f}Mbps PDR={result.pdr:.1f}% D={result.delay*1000:.1f}ms")
            
            all_results[protocol][num_nodes] = runs
    
    # Aggregate results
    aggregated = aggregate_all_results(all_results)
    
    # Save CSV
    save_csv(aggregated)
    
    # Generate graphs
    generate_graphs(aggregated)
    
    elapsed_total = (time.time() - start_time) / 60
    print(f"\n{'='*70}")
    print(f"  TESTING COMPLETE! Total time: {elapsed_total:.1f} minutes")
    print(f"{'='*70}")

def aggregate_all_results(all_results):
    """Aggregate multiple runs into mean/std"""
    aggregated = {}
    
    for protocol in PROTOCOLS:
        aggregated[protocol] = {}
        for nodes, runs in all_results[protocol].items():
            throughputs = [r.throughput for r in runs]
            pdrs = [r.pdr for r in runs]
            delays = [r.delay for r in runs]
            
            aggregated[protocol][nodes] = {
                'throughput_mean': np.mean(throughputs),
                'throughput_std': np.std(throughputs),
                'pdr_mean': np.mean(pdrs),
                'pdr_std': np.std(pdrs),
                'delay_mean': np.mean(delays),
                'delay_std': np.std(delays),
            }
    
    return aggregated

def save_csv(aggregated):
    """Save results to CSV"""
    csv_file = os.path.join(OUTPUT_DIR, "scalability_test_results.csv")
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Nodes", "Protocol", "Throughput_Mean", "Throughput_Std",
                         "PDR_Mean", "PDR_Std", "Delay_Mean_ms", "Delay_Std_ms"])
        
        for nodes in NODE_COUNTS:
            for protocol in PROTOCOLS:
                if nodes in aggregated[protocol]:
                    d = aggregated[protocol][nodes]
                    writer.writerow([
                        nodes, protocol,
                        f"{d['throughput_mean']:.4f}", f"{d['throughput_std']:.4f}",
                        f"{d['pdr_mean']:.2f}", f"{d['pdr_std']:.2f}",
                        f"{d['delay_mean']*1000:.2f}", f"{d['delay_std']*1000:.2f}"
                    ])
    
    print(f"\nResults saved to: {csv_file}")

def generate_graphs(aggregated):
    """Generate publication-quality comparison graphs"""
    
    # Style configuration
    plt.style.use('seaborn-v0_8-whitegrid')
    colors = {'pgh': '#2ecc71', 'cr-ed': '#9b59b6', 'aodv': '#e74c3c', 'olsr': '#3498db'}
    markers = {'pgh': 'o', 'cr-ed': 'D', 'aodv': '^', 'olsr': 's'}
    labels = {'pgh': 'PGH-LMSS (Cross-Layer)', 'cr-ed': 'CR-ED', 'aodv': 'AODV', 'olsr': 'OLSR'}
    
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('FANET Protocol Scalability Comparison (6-100 Nodes)', fontsize=14, fontweight='bold')
    
    # ===== THROUGHPUT GRAPH =====
    ax1 = axes[0]
    ax1.set_title('Throughput vs Number of Nodes', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Number of Nodes', fontsize=11)
    ax1.set_ylabel('Throughput (Mbps)', fontsize=11)
    
    for protocol in PROTOCOLS:
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['throughput_mean'] for n in nodes]
        stds = [aggregated[protocol][n]['throughput_std'] for n in nodes]
        
        ax1.errorbar(
            nodes, means, yerr=stds,
            marker=markers[protocol],
            color=colors[protocol],
            label=labels[protocol],
            linewidth=2,
            markersize=7,
            capsize=3,
            capthick=1.5
        )
    
    ax1.legend(loc='best', fontsize=9)
    ax1.set_xlim(0, 105)
    ax1.grid(True, alpha=0.3)
    
    # ===== PDR GRAPH =====
    ax2 = axes[1]
    ax2.set_title('Packet Delivery Ratio vs Number of Nodes', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Number of Nodes', fontsize=11)
    ax2.set_ylabel('PDR (%)', fontsize=11)
    
    for protocol in PROTOCOLS:
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['pdr_mean'] for n in nodes]
        stds = [aggregated[protocol][n]['pdr_std'] for n in nodes]
        
        ax2.errorbar(
            nodes, means, yerr=stds,
            marker=markers[protocol],
            color=colors[protocol],
            label=labels[protocol],
            linewidth=2,
            markersize=7,
            capsize=3,
            capthick=1.5
        )
    
    ax2.legend(loc='best', fontsize=9)
    ax2.set_xlim(0, 105)
    ax2.set_ylim(0, 105)
    ax2.grid(True, alpha=0.3)
    
    # ===== DELAY GRAPH =====
    ax3 = axes[2]
    ax3.set_title('End-to-End Delay vs Number of Nodes', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Number of Nodes', fontsize=11)
    ax3.set_ylabel('Delay (ms)', fontsize=11)
    
    for protocol in PROTOCOLS:
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['delay_mean'] * 1000 for n in nodes]  # Convert to ms
        stds = [aggregated[protocol][n]['delay_std'] * 1000 for n in nodes]
        
        ax3.errorbar(
            nodes, means, yerr=stds,
            marker=markers[protocol],
            color=colors[protocol],
            label=labels[protocol],
            linewidth=2,
            markersize=7,
            capsize=3,
            capthick=1.5
        )
    
    ax3.legend(loc='best', fontsize=9)
    ax3.set_xlim(0, 105)
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save graph
    graph_file = os.path.join(OUTPUT_DIR, "scalability_comparison.png")
    plt.savefig(graph_file, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"Graph saved to: {graph_file}")
    
    plt.close()

if __name__ == "__main__":
    main()
