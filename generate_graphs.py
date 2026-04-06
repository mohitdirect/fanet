#!/usr/bin/env python3
"""
Generate graphs from collected scalability test results
Parses the log file and creates comparison graphs
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import os

# Parse the log file
LOG_FILE = "/home/mohit/ns-allinone-3.43/ns-3.43/scratch/myfanet/scalability_output.log"
OUTPUT_DIR = "/home/mohit/ns-allinone-3.43/ns-3.43/scratch/myfanet"

def parse_results():
    """Parse the log file and extract results"""
    results = defaultdict(lambda: defaultdict(list))
    
    with open(LOG_FILE, 'r') as f:
        content = f.read()
    
    # Pattern: [X/144] PROTOCOL n=XX run=X (ETA: Xm)... T=X.XXMbps PDR=X.X% D=X.Xms
    pattern = r'\[\s*\d+/\d+\]\s+(\w+(?:-\w+)?)\s+n=\s*(\d+)\s+run=\d+.*T=([0-9.]+)Mbps\s+PDR=([0-9.]+)%\s+D=([0-9.]+)ms'
    
    for match in re.finditer(pattern, content):
        protocol = match.group(1).lower()
        nodes = int(match.group(2))
        throughput = float(match.group(3))
        pdr = float(match.group(4))
        delay = float(match.group(5))
        
        results[protocol][nodes].append({
            'throughput': throughput,
            'pdr': pdr,
            'delay': delay
        })
    
    return results

def aggregate_results(results):
    """Compute mean and std for each protocol/node combination"""
    aggregated = {}
    
    for protocol, node_data in results.items():
        aggregated[protocol] = {}
        for nodes, runs in node_data.items():
            if runs:  # Only if we have data
                throughputs = [r['throughput'] for r in runs]
                pdrs = [r['pdr'] for r in runs]
                delays = [r['delay'] for r in runs]
                
                aggregated[protocol][nodes] = {
                    'throughput_mean': np.mean(throughputs),
                    'throughput_std': np.std(throughputs) if len(throughputs) > 1 else 0,
                    'pdr_mean': np.mean(pdrs),
                    'pdr_std': np.std(pdrs) if len(pdrs) > 1 else 0,
                    'delay_mean': np.mean(delays),
                    'delay_std': np.std(delays) if len(delays) > 1 else 0,
                }
    
    return aggregated

def generate_graphs(aggregated):
    """Generate publication-quality comparison graphs"""
    
    # Style configuration
    plt.style.use('seaborn-v0_8-whitegrid')
    colors = {'pgh': '#2ecc71', 'cr-ed': '#9b59b6', 'aodv': '#e74c3c', 'olsr': '#3498db'}
    markers = {'pgh': 'o', 'cr-ed': 'D', 'aodv': '^', 'olsr': 's'}
    labels = {'pgh': 'PGH-LMSS (Cross-Layer)', 'cr-ed': 'CR-ED', 'aodv': 'AODV', 'olsr': 'OLSR'}
    
    protocols = ['pgh', 'cr-ed', 'aodv', 'olsr']
    
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('FANET Protocol Scalability Comparison (6-100 Nodes)', fontsize=14, fontweight='bold')
    
    # ===== THROUGHPUT GRAPH =====
    ax1 = axes[0]
    ax1.set_title('Throughput vs Number of Nodes', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Number of Nodes', fontsize=11)
    ax1.set_ylabel('Throughput (Mbps)', fontsize=11)
    
    for protocol in protocols:
        if protocol not in aggregated:
            continue
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['throughput_mean'] for n in nodes]
        stds = [aggregated[protocol][n]['throughput_std'] for n in nodes]
        
        ax1.errorbar(
            nodes, means, yerr=stds,
            marker=markers.get(protocol, 'o'),
            color=colors.get(protocol, 'gray'),
            label=labels.get(protocol, protocol.upper()),
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
    
    for protocol in protocols:
        if protocol not in aggregated:
            continue
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['pdr_mean'] for n in nodes]
        stds = [aggregated[protocol][n]['pdr_std'] for n in nodes]
        
        ax2.errorbar(
            nodes, means, yerr=stds,
            marker=markers.get(protocol, 'o'),
            color=colors.get(protocol, 'gray'),
            label=labels.get(protocol, protocol.upper()),
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
    
    for protocol in protocols:
        if protocol not in aggregated:
            continue
        nodes = sorted(aggregated[protocol].keys())
        means = [aggregated[protocol][n]['delay_mean'] for n in nodes]
        stds = [aggregated[protocol][n]['delay_std'] for n in nodes]
        
        ax3.errorbar(
            nodes, means, yerr=stds,
            marker=markers.get(protocol, 'o'),
            color=colors.get(protocol, 'gray'),
            label=labels.get(protocol, protocol.upper()),
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
    return graph_file

def save_csv(aggregated):
    """Save results to CSV"""
    import csv
    
    csv_file = os.path.join(OUTPUT_DIR, "scalability_test_results.csv")
    
    all_nodes = set()
    for protocol in aggregated:
        all_nodes.update(aggregated[protocol].keys())
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Nodes", "Protocol", "Throughput_Mean", "Throughput_Std",
                         "PDR_Mean", "PDR_Std", "Delay_Mean_ms", "Delay_Std_ms"])
        
        for nodes in sorted(all_nodes):
            for protocol in ['pgh', 'cr-ed', 'aodv', 'olsr']:
                if protocol in aggregated and nodes in aggregated[protocol]:
                    d = aggregated[protocol][nodes]
                    writer.writerow([
                        nodes, protocol,
                        f"{d['throughput_mean']:.4f}", f"{d['throughput_std']:.4f}",
                        f"{d['pdr_mean']:.2f}", f"{d['pdr_std']:.2f}",
                        f"{d['delay_mean']:.2f}", f"{d['delay_std']:.2f}"
                    ])
    
    print(f"CSV saved to: {csv_file}")
    return csv_file

def print_summary(aggregated):
    """Print summary table"""
    print("\n" + "="*80)
    print("  PROTOCOL COMPARISON SUMMARY")
    print("="*80)
    
    protocols = ['pgh', 'cr-ed', 'aodv', 'olsr']
    
    # Get all node counts
    all_nodes = set()
    for p in protocols:
        if p in aggregated:
            all_nodes.update(aggregated[p].keys())
    
    print(f"\n{'Nodes':<8}", end="")
    for p in protocols:
        print(f"{p.upper():<20}", end="")
    print()
    print("-"*80)
    
    for nodes in sorted(all_nodes):
        print(f"{nodes:<8}", end="")
        for p in protocols:
            if p in aggregated and nodes in aggregated[p]:
                d = aggregated[p][nodes]
                print(f"T:{d['throughput_mean']:.2f} P:{d['pdr_mean']:.0f}%    ", end="")
            else:
                print(f"{'N/A':<20}", end="")
        print()
    
    print("-"*80)

if __name__ == "__main__":
    print("Parsing log file...")
    results = parse_results()
    
    print("Aggregating results...")
    aggregated = aggregate_results(results)
    
    print_summary(aggregated)
    
    print("\nGenerating graphs...")
    generate_graphs(aggregated)
    
    print("Saving CSV...")
    save_csv(aggregated)
    
    print("\n✅ Done!")
