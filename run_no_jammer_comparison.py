#!/usr/bin/env python3
"""
No-Jammer Protocol Comparison — Baseline Performance Test
Runs PGH, GPSR, CBRP, CR-ED, AODV, OLSR across 10-100 nodes (step 2)
WITHOUT any jamming, to establish a clean baseline for comparison.

Identical behaviour to run_comparison_tests.py:
- 5 seeds per configuration
- Checkpoint/resume (survives power failures)
- Multiprocessing per node-count batch
- Graphs updated after every node batch
- 4 metrics: Throughput, PDR, End-to-End Delay, Jitter
- Results saved to results/no_jammer/
"""

import multiprocessing
import subprocess
import re
import os
import csv
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend (safe for multiprocessing)
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, List, Tuple, Set
from dataclasses import dataclass, field

# ─── Configuration ────────────────────────────────────────────────────────────
NS3_DIR        = "/home/mohit/ns-allinone-3.43/ns-3.43"
SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR    = os.path.join(SCRIPT_DIR, "results", "no_jammer")

PROTOCOLS      = ["pgh", "gpsr", "cbrp", "cr-ed", "aodv", "olsr"]
NODE_RANGE     = range(10, 102, 2)   # 10, 12, 14, ..., 100
TRAFFIC_MODE   = "cluster"
DETECTION_MODE = "hybrid"

# Jammer is effectively disabled by pushing start time far beyond sim end
JAMMER_MODE    = "static"
JAMMER_START   = 9999   # Never activates

# Timing
SIM_TIME       = 30

# Statistical
RUNS_PER_CONFIG = 5
TEST_SEEDS      = [3, 7, 42, 123, 999]

# Plot style
COLORS = {
    'pgh':   '#2ecc71',
    'gpsr':  '#f39c12',
    'cbrp':  '#1abc9c',
    'cr-ed': '#3498db',
    'aodv':  '#e74c3c',
    'olsr':  '#9b59b6',
}
MARKERS = {
    'pgh':   'o',
    'gpsr':  'p',
    'cbrp':  'h',
    'cr-ed': 's',
    'aodv':  '^',
    'olsr':  'd',
}
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class SimulationResult:
    protocol:   str   = ""
    nodes:      int   = 0
    run:        int   = 0
    throughput: float = 0.0
    pdr:        float = 0.0
    delay:      float = 0.0
    jitter:     float = 0.0


@dataclass
class AggregatedResult:
    throughput_mean: float = 0.0
    throughput_std:  float = 0.0
    pdr_mean:        float = 0.0
    pdr_std:         float = 0.0
    delay_mean:      float = 0.0
    delay_std:       float = 0.0
    jitter_mean:     float = 0.0
    jitter_std:      float = 0.0
    runs: List[SimulationResult] = field(default_factory=list)


# ─── Simulation runner ────────────────────────────────────────────────────────
def run_simulation(args: Tuple[str, int, int]) -> SimulationResult:
    """Run a single NS-3 simulation (no jammer) and parse metrics."""
    protocol, num_nodes, run = args
    seed = TEST_SEEDS[run % len(TEST_SEEDS)]

    cmd = [
        "./ns3", "run",
        (
            f"scratch/myfanet/pgh"
            f" --protocol={protocol}"
            f" --nNodes={num_nodes}"
            f" --traffic={TRAFFIC_MODE}"
            f" --detection={DETECTION_MODE}"
            f" --jammer={JAMMER_MODE}"
            f" --run={run}"
            f" --seed={seed}"
            f" --simTime={SIM_TIME}"
            f" --jammerStart={JAMMER_START}"
        )
    ]

    result = SimulationResult(protocol=protocol, nodes=num_nodes, run=run)

    try:
        proc = subprocess.run(
            cmd,
            cwd=NS3_DIR,
            capture_output=True,
            text=True,
            timeout=600,
        )
        output = proc.stdout + proc.stderr

        m = re.search(r'Total Throughput.*?:\s*([\d.]+)\s*Mbps', output)
        if m:
            result.throughput = float(m.group(1))

        m = re.search(r'Packet Delivery Ratio.*?:\s*([\d.]+)\s*%', output)
        if m:
            result.pdr = float(m.group(1))

        m = re.search(r'Avg End-to-End Delay.*?:\s*([\d.]+)\s*s', output)
        if m:
            result.delay = float(m.group(1))

        m = re.search(r'Avg Jitter.*?:\s*([\d.]+)\s*s', output)
        if m:
            result.jitter = float(m.group(1))

    except Exception as e:
        print(f"[ERROR] {protocol} {num_nodes}n run{run}: {e}")

    return result


# ─── Aggregation ──────────────────────────────────────────────────────────────
def aggregate_results(run_list: List[SimulationResult]) -> AggregatedResult:
    agg = AggregatedResult()
    agg.runs             = run_list
    agg.throughput_mean  = float(np.mean([r.throughput for r in run_list]))
    agg.throughput_std   = float(np.std ([r.throughput for r in run_list]))
    agg.pdr_mean         = float(np.mean([r.pdr        for r in run_list]))
    agg.pdr_std          = float(np.std ([r.pdr        for r in run_list]))
    agg.delay_mean       = float(np.mean([r.delay      for r in run_list]))
    agg.delay_std        = float(np.std ([r.delay      for r in run_list]))
    agg.jitter_mean      = float(np.mean([r.jitter     for r in run_list]))
    agg.jitter_std       = float(np.std ([r.jitter     for r in run_list]))
    return agg


# ─── File paths ───────────────────────────────────────────────────────────────
def detailed_csv_path() -> str:
    return os.path.join(RESULTS_DIR, "no_jammer_detailed.csv")

def summary_csv_path() -> str:
    return os.path.join(RESULTS_DIR, "no_jammer_results.csv")

def graph_path() -> str:
    return os.path.join(RESULTS_DIR, "no_jammer_graph.png")


# ─── Checkpoint ───────────────────────────────────────────────────────────────
def load_checkpoints() -> Set[Tuple[str, int, int]]:
    completed: Set[Tuple[str, int, int]] = set()
    path = detailed_csv_path()
    if not os.path.exists(path):
        return completed
    try:
        with open(path, 'r') as f:
            for row in csv.DictReader(f):
                completed.add((row['Protocol'], int(row['Nodes']), int(row['Run'])))
        print(f"[Checkpoint] Loaded {len(completed)} existing results.")
    except Exception as e:
        print(f"[Warning] Could not load checkpoints: {e}")
    return completed


def append_results_to_csv(results: List[SimulationResult]):
    path = detailed_csv_path()
    write_header = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["Nodes", "Protocol", "Run",
                             "Throughput", "PDR", "Delay", "Jitter"])
        for r in results:
            writer.writerow([
                r.nodes, r.protocol, r.run,
                f"{r.throughput:.4f}", f"{r.pdr:.2f}",
                f"{r.delay:.6f}",     f"{r.jitter:.6f}",
            ])


# ─── Re-aggregate from CSV ────────────────────────────────────────────────────
def read_all_results() -> Dict[str, Dict[int, AggregatedResult]]:
    flat: List[SimulationResult] = []
    path = detailed_csv_path()
    if not os.path.exists(path):
        return {p: {} for p in PROTOCOLS}

    with open(path, 'r') as f:
        for row in csv.DictReader(f):
            try:
                flat.append(SimulationResult(
                    protocol   = row['Protocol'],
                    nodes      = int(row['Nodes']),
                    run        = int(row['Run']),
                    throughput = float(row['Throughput']),
                    pdr        = float(row['PDR']),
                    delay      = float(row['Delay']),
                    jitter     = float(row.get('Jitter', 0)),
                ))
            except Exception:
                pass

    nodes_found = sorted(set(r.nodes for r in flat))
    grouped: Dict[str, Dict[int, List[SimulationResult]]] = {
        p: {n: [] for n in nodes_found} for p in PROTOCOLS
    }
    for r in flat:
        if r.protocol in grouped and r.nodes in grouped[r.protocol]:
            grouped[r.protocol][r.nodes].append(r)

    final: Dict[str, Dict[int, AggregatedResult]] = {p: {} for p in PROTOCOLS}
    for p in PROTOCOLS:
        for n in nodes_found:
            if grouped[p][n]:
                final[p][n] = aggregate_results(grouped[p][n])
    return final


def save_summary_csv(final: Dict[str, Dict[int, AggregatedResult]]):
    with open(summary_csv_path(), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Nodes", "Protocol",
                         "Throughput_Mean", "Throughput_Std",
                         "PDR_Mean",        "PDR_Std",
                         "Delay_Mean",      "Delay_Std",
                         "Jitter_Mean",     "Jitter_Std"])
        for p in PROTOCOLS:
            for n in sorted(final[p].keys()):
                agg = final[p][n]
                writer.writerow([
                    n, p,
                    f"{agg.throughput_mean:.4f}", f"{agg.throughput_std:.4f}",
                    f"{agg.pdr_mean:.2f}",        f"{agg.pdr_std:.2f}",
                    f"{agg.delay_mean:.6f}",      f"{agg.delay_std:.6f}",
                    f"{agg.jitter_mean:.6f}",     f"{agg.jitter_std:.6f}",
                ])


# ─── Plotting ─────────────────────────────────────────────────────────────────
def plot_graphs(final: Dict[str, Dict[int, AggregatedResult]]):
    fig, axes = plt.subplots(2, 2, figsize=(16, 13))
    fig.suptitle(
        'FANET Protocol Comparison — No Jammer Baseline (10–100 Nodes, 5 Seeds)',
        fontsize=15, fontweight='bold', y=1.01
    )

    def _draw(ax, metric_fn, title, ylabel, ylim=None):
        for p in PROTOCOLS:
            nodes = sorted(final[p].keys())
            if not nodes:
                continue
            means, stds = zip(*[metric_fn(final[p][n]) for n in nodes])
            ax.errorbar(
                nodes, means, yerr=stds,
                marker=MARKERS[p], color=COLORS[p],
                label=p.upper(), linewidth=2,
                markersize=7, capsize=3, capthick=1.5,
            )
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xlabel('Number of Nodes', fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        if ylim:
            ax.set_ylim(ylim)

    _draw(axes[0, 0],
          lambda a: (a.throughput_mean, a.throughput_std),
          'Average Throughput vs Number of Nodes (No Jammer)',
          'Throughput (Mbps)')

    _draw(axes[0, 1],
          lambda a: (a.pdr_mean, a.pdr_std),
          'Packet Delivery Ratio vs Number of Nodes (No Jammer)',
          'PDR (%)', ylim=(0, 105))

    _draw(axes[1, 0],
          lambda a: (a.delay_mean * 1000, a.delay_std * 1000),
          'Average End-to-End Delay vs Number of Nodes (No Jammer)',
          'Delay (ms)')

    _draw(axes[1, 1],
          lambda a: (a.jitter_mean * 1000, a.jitter_std * 1000),
          'Average Jitter vs Number of Nodes (No Jammer)',
          'Jitter (ms)')

    plt.tight_layout()
    out = graph_path()
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"[Graph] Saved → {out}")
    plt.close(fig)


def refresh_plots():
    final = read_all_results()
    if not any(final[p] for p in PROTOCOLS):
        return
    save_summary_csv(final)
    try:
        plot_graphs(final)
    except Exception as e:
        print(f"[Warning] Plot failed: {e}")


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 65)
    print("NO-JAMMER BASELINE COMPARISON (10–100 Nodes, Step 2)")
    print(f"Protocols  : {', '.join(PROTOCOLS)}")
    print(f"Seeds      : {TEST_SEEDS}")
    print(f"Sim time   : {SIM_TIME}s   Jammer: DISABLED")
    print(f"Results dir: {RESULTS_DIR}")
    print("=" * 65)

    completed  = load_checkpoints()
    num_cores  = multiprocessing.cpu_count()
    print(f"CPU cores available: {num_cores}\n")

    for nodes in NODE_RANGE:
        node_tasks = [
            (proto, nodes, run)
            for proto in PROTOCOLS
            for run in range(1, RUNS_PER_CONFIG + 1)
            if (proto, nodes, run) not in completed
        ]

        total_expected = len(PROTOCOLS) * RUNS_PER_CONFIG
        skipped = total_expected - len(node_tasks)

        if not node_tasks:
            print(f"[Nodes={nodes:3d}] All {total_expected} runs already complete — skipping.")
            continue

        print(f"\n[Nodes={nodes:3d}] Running {len(node_tasks)} sims "
              f"(skipped {skipped} checkpointed) with {num_cores} workers...")

        batch_results: List[SimulationResult] = []
        with multiprocessing.Pool(processes=num_cores) as pool:
            for i, result in enumerate(pool.imap_unordered(run_simulation, node_tasks)):
                batch_results.append(result)
                completed.add((result.protocol, result.nodes, result.run))
                print(
                    f"  [{i+1:2d}/{len(node_tasks)}] "
                    f"{result.protocol.upper():6s} n={result.nodes} "
                    f"seed={TEST_SEEDS[result.run % len(TEST_SEEDS)]} → "
                    f"T={result.throughput:.3f} Mbps  "
                    f"PDR={result.pdr:.1f}%  "
                    f"Delay={result.delay*1000:.1f}ms  "
                    f"Jitter={result.jitter*1000:.1f}ms"
                )

        append_results_to_csv(batch_results)
        print(f"[Nodes={nodes:3d}] CSV updated. Regenerating graphs...")
        refresh_plots()

    print("\n" + "=" * 65)
    print("No-jammer baseline sweep complete!")
    print("=" * 65)
    refresh_plots()
    print(f"\nResults folder : {RESULTS_DIR}")
    print(f"  Detailed CSV : {detailed_csv_path()}")
    print(f"  Summary CSV  : {summary_csv_path()}")
    print(f"  Graph        : {graph_path()}")


if __name__ == "__main__":
    main()
