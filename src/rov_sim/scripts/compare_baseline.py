#!/usr/bin/env python3
"""
compare_baseline.py — Compute control-performance metrics from a rosbag
and compare against a saved baseline.

Usage:
    python3 compare_baseline.py <rosbag_directory> [--baseline <path>]

Metrics computed:
    - Overshoot (%):       max(depth) relative to target, as a percentage
    - Settling time (s):   time for depth to enter and stay within ±5% of target
    - Steady-state error:  mean |depth - target| over the final 20% of the run
    - Rise time (s):       time from 10% to 90% of target depth

If a baseline JSON exists, each metric is compared within configurable
tolerances. If no baseline exists, the current metrics become the new baseline.

Exit codes:
    0 — all metrics within tolerance (PASS) or new baseline saved
    1 — one or more metrics outside tolerance (FAIL)
"""

import json
import math
import os
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# rosbag2_py import (available in any ROS 2 install)
# ---------------------------------------------------------------------------
try:
    import rosbag2_py
except ImportError:
    print("ERROR: rosbag2_py not found. Make sure your ROS 2 workspace is sourced.",
          file=sys.stderr)
    sys.exit(1)

# We need deserialization utilities from rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ---------------------------------------------------------------------------
# Configuration — Tolerance thresholds for baseline comparison
# ---------------------------------------------------------------------------
TOLERANCES = {
    "overshoot_pct":     10.0,   # ±10 percentage-points
    "settling_time_s":    1.0,   # ±1.0 seconds
    "ss_error_m":         0.05,  # ±0.05 meters
    "rise_time_s":        1.0,   # ±1.0 seconds
}

SETTLING_BAND = 0.05   # 5% band for settling-time calculation
FINAL_FRACTION = 0.20  # last 20% of samples for steady-state error


# ---------------------------------------------------------------------------
# Bag reading helpers
# ---------------------------------------------------------------------------
def read_messages(bag_path: str, topic: str):
    """Yield (timestamp_ns, deserialized_msg) for every message on `topic`."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Build a filter for just our topic
    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    # Resolve the message type for this topic from the bag metadata
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        return  # topic not in bag — caller handles empty result

    msg_type = get_message(type_map[topic])

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        msg = deserialize_message(data, msg_type)
        yield timestamp_ns, msg


def extract_depth_timeseries(bag_path: str):
    """Return list of (time_s, depth_m) from /sensor_data."""
    samples = []
    t0 = None
    for ts_ns, msg in read_messages(bag_path, "/sensor_data"):
        if t0 is None:
            t0 = ts_ns
        t_sec = (ts_ns - t0) * 1e-9
        samples.append((t_sec, float(msg.depth)))
    return samples


def extract_target_depth(bag_path: str):
    """Return the last commanded heave setpoint (index 5) from /setpoints."""
    last_heave = None
    for _ts_ns, msg in read_messages(bag_path, "/setpoints"):
        last_heave = float(msg.setpoints[5])
    return last_heave


# ---------------------------------------------------------------------------
# Metric calculations
# ---------------------------------------------------------------------------
def compute_metrics(depth_series, target):
    """
    Compute control-performance metrics.

    Parameters
    ----------
    depth_series : list of (time_s, depth_m)
    target : float — target depth (heave setpoint)

    Returns
    -------
    dict with keys: overshoot_pct, settling_time_s, ss_error_m, rise_time_s
    """
    if not depth_series or target is None:
        print("ERROR: Insufficient data to compute metrics.", file=sys.stderr)
        sys.exit(1)

    times  = [s[0] for s in depth_series]
    depths = [s[1] for s in depth_series]

    # --- Overshoot (%) ---
    # Overshoot is the maximum excursion beyond the target.
    # For a positive target (diving down), overshoot = max(depth) - target.
    # We use the absolute sense so it also works for negative targets.
    if abs(target) < 1e-9:
        overshoot_pct = 0.0
    else:
        if target > 0:
            peak = max(depths)
            overshoot_pct = max(0.0, (peak - target) / target * 100.0)
        else:
            peak = min(depths)
            overshoot_pct = max(0.0, (target - peak) / abs(target) * 100.0)

    # --- Rise time (10% → 90% of target) ---
    thresh_10 = 0.10 * target
    thresh_90 = 0.90 * target
    t_10 = None
    t_90 = None

    # Determine direction: are we going from 0 towards target?
    sign = 1.0 if target >= 0 else -1.0
    for t, d in depth_series:
        if t_10 is None and (sign * d) >= (sign * thresh_10):
            t_10 = t
        if t_90 is None and (sign * d) >= (sign * thresh_90):
            t_90 = t
        if t_10 is not None and t_90 is not None:
            break

    rise_time_s = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else float("nan")

    # --- Settling time (time to enter and stay within ±5% of target) ---
    band = abs(target) * SETTLING_BAND
    settling_time_s = float("nan")
    # Walk backwards to find the last time we were outside the band
    for i in range(len(depth_series) - 1, -1, -1):
        if abs(depths[i] - target) > band:
            # The settling time is the next sample's time
            if i + 1 < len(depth_series):
                settling_time_s = times[i + 1]
            break
    else:
        # Never left the band (or started inside) — settled at t=0
        settling_time_s = 0.0

    # --- Steady-state error (mean |depth - target| over final 20%) ---
    n_final = max(1, int(len(depths) * FINAL_FRACTION))
    final_depths = depths[-n_final:]
    ss_error_m = sum(abs(d - target) for d in final_depths) / len(final_depths)

    return {
        "overshoot_pct":  round(overshoot_pct, 4),
        "settling_time_s": round(settling_time_s, 4) if not math.isnan(settling_time_s) else None,
        "ss_error_m":      round(ss_error_m, 6),
        "rise_time_s":     round(rise_time_s, 4) if not math.isnan(rise_time_s) else None,
    }


# ---------------------------------------------------------------------------
# Baseline comparison
# ---------------------------------------------------------------------------
def compare_with_baseline(metrics, baseline, tolerances):
    """
    Compare metrics against baseline within tolerances.

    Returns (overall_pass: bool, results: list[dict])
    """
    results = []
    overall_pass = True

    for key, tol in tolerances.items():
        current = metrics.get(key)
        base    = baseline.get(key)

        # Skip if either value is None (metric couldn't be computed)
        if current is None or base is None:
            results.append({
                "metric": key,
                "status": "SKIP",
                "current": current,
                "baseline": base,
                "tolerance": tol,
                "delta": None,
            })
            continue

        delta = abs(current - base)
        passed = delta <= tol
        if not passed:
            overall_pass = False

        results.append({
            "metric": key,
            "status": "PASS" if passed else "FAIL",
            "current": current,
            "baseline": base,
            "tolerance": tol,
            "delta": round(delta, 6),
        })

    return overall_pass, results


def print_results_table(results):
    """Pretty-print comparison results."""
    hdr = f"{'Metric':<20s} {'Status':<8s} {'Current':>12s} {'Baseline':>12s} {'Delta':>10s} {'Tolerance':>10s}"
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        cur_str  = f"{r['current']:.4f}" if r["current"] is not None else "N/A"
        base_str = f"{r['baseline']:.4f}" if r["baseline"] is not None else "N/A"
        dlt_str  = f"{r['delta']:.4f}" if r["delta"] is not None else "N/A"
        tol_str  = f"±{r['tolerance']:.4f}"
        status_icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️ "}.get(r["status"], "")
        print(f"{r['metric']:<20s} {status_icon} {r['status']:<5s} {cur_str:>12s} {base_str:>12s} {dlt_str:>10s} {tol_str:>10s}")


def print_metrics(metrics):
    """Pretty-print a single set of metrics."""
    print(f"  {'Metric':<20s} {'Value':>12s}")
    print(f"  {'-'*20} {'-'*12}")
    for key, val in metrics.items():
        val_str = f"{val:.4f}" if val is not None else "N/A"
        print(f"  {key:<20s} {val_str:>12s}")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Compare rosbag control metrics against a baseline."
    )
    parser.add_argument(
        "bag_dir",
        help="Path to the rosbag2 directory (SQLite3 format).",
    )
    parser.add_argument(
        "--baseline",
        default=None,
        help="Path to baseline JSON. Default: <bag_dir>/baseline_metrics.json",
    )
    args = parser.parse_args()

    bag_dir = os.path.abspath(args.bag_dir)
    baseline_path = args.baseline or os.path.join(bag_dir, "baseline_metrics.json")

    if not os.path.isdir(bag_dir):
        print(f"ERROR: Bag directory not found: {bag_dir}", file=sys.stderr)
        sys.exit(1)

    # ------------------------------------------------------------------
    # Extract data
    # ------------------------------------------------------------------
    print(f"[compare] Reading bag: {bag_dir}")

    depth_series = extract_depth_timeseries(bag_dir)
    if not depth_series:
        print("ERROR: No /sensor_data messages found in bag.", file=sys.stderr)
        sys.exit(1)
    print(f"[compare] Found {len(depth_series)} /sensor_data samples "
          f"over {depth_series[-1][0]:.2f}s")

    target = extract_target_depth(bag_dir)
    if target is None:
        print("ERROR: No /setpoints messages found in bag.", file=sys.stderr)
        sys.exit(1)
    print(f"[compare] Target depth (heave setpoint): {target:.3f} m")

    # ------------------------------------------------------------------
    # Compute metrics
    # ------------------------------------------------------------------
    metrics = compute_metrics(depth_series, target)
    print("\n[compare] Computed metrics:")
    print_metrics(metrics)
    print()

    # ------------------------------------------------------------------
    # Baseline comparison or creation
    # ------------------------------------------------------------------
    if os.path.isfile(baseline_path):
        # --- Compare against existing baseline ---
        print(f"[compare] Loading baseline: {baseline_path}")
        with open(baseline_path, "r") as f:
            baseline = json.load(f)

        overall_pass, results = compare_with_baseline(metrics, baseline, TOLERANCES)

        print("\n[compare] Comparison results:")
        print_results_table(results)
        print()

        if overall_pass:
            print("[compare] ✅ OVERALL: PASS — all metrics within tolerance.")
            return 0
        else:
            print("[compare] ❌ OVERALL: FAIL — one or more metrics outside tolerance.")
            return 1

    else:
        # --- Save as new baseline ---
        print(f"[compare] INFO: No baseline found at {baseline_path}")
        print("[compare] Saving current metrics as the new baseline.")

        # Ensure the parent directory exists
        os.makedirs(os.path.dirname(baseline_path), exist_ok=True)

        with open(baseline_path, "w") as f:
            json.dump(metrics, f, indent=2)
            f.write("\n")

        print(f"[compare] Baseline saved to: {baseline_path}")
        return 0


if __name__ == "__main__":
    sys.exit(main())
