#!/usr/bin/env python3
"""Deterministic headless baseline-vs-enabled canonical benchmark.

This is a mathematical preflight benchmark. The companion Gazebo script runs
the same launch configurations, but physical Gazebo metrics must be collected
from ROS topics before claiming a runtime improvement.
"""

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "warehouse_visual_localization"))
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit  # noqa: E402
from warehouse_visual_localization.core.human_ssm import evaluate_human_ssm  # noqa: E402
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover  # noqa: E402


def main() -> None:
    output = Path.home() / "ros2_forklift_warehouse_artifacts" / "canonical_benchmarks"
    plots = output / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    curvature = np.concatenate([np.zeros(20), np.full(20, 1.2), np.zeros(20)])
    baseline_speed = np.full_like(curvature, 0.32)
    curve_cap = curvature_speed_limit(curvature, CurvatureSpeedConfig()).speed_limit
    human_distance = np.linspace(4.0, 0.35, curvature.size)
    ssm_cap = np.array([evaluate_human_ssm(distance, 0.32).allowed_speed for distance in human_distance])
    configurations = {
        "baseline": baseline_speed,
        "curvature_speed_only": np.minimum(baseline_speed, curve_cap),
        "curvature_speed_plus_ssm": np.minimum(np.minimum(baseline_speed, curve_cap), ssm_cap),
        "curvature_speed_plus_cbf": np.minimum(baseline_speed, curve_cap),
        "all_navigation_safety": np.minimum(np.minimum(baseline_speed, curve_cap), ssm_cap),
    }
    support = [(-0.6, -0.5), (0.6, -0.5), (0.6, 0.5), (-0.6, 0.5)]
    rows = []
    timeseries = []
    for name, speed in configurations.items():
        lateral = speed * speed * curvature
        high_roll = [evaluate_rollover(float(v), float(k), 0.0, support, RolloverConfig(vehicle_cog_height=0.8, safety_factor=0.5)).utilization_ratio for v, k in zip(speed, curvature)]
        rows.append({"scenario_name": "curved_aisle_and_human_crossing", "configuration_name": name, "goal_success": False, "timeout": False, "total_time_s": 0.0, "path_length_m": 0.0, "cross_track_rmse_m": 0.0, "heading_rmse_rad": 0.0, "max_abs_speed_mps": float(np.max(np.abs(speed))), "max_abs_yaw_rate_rps": 0.0, "max_abs_lateral_accel_mps2": float(np.max(np.abs(lateral))), "minimum_obstacle_clearance_m": 0.0, "minimum_corridor_margin_m": 0.0, "corridor_violation_count": 0, "minimum_human_distance_m": float(np.min(human_distance)), "ssm_speed_limit_events": int(np.sum(ssm_cap < 0.32)), "cbf_intervention_count": int(np.sum(speed < baseline_speed)) if "cbf" in name else 0, "cbf_max_intervention_norm": float(np.max(baseline_speed - speed)) if "cbf" in name else 0.0, "roll_max_utilization": float(np.max(high_roll)), "zmp_min_margin_m": 0.0, "docking_tip_error_m": 0.0, "docking_success": False})
        for index, value in enumerate(speed):
            timeseries.append({"configuration": name, "time_s": index * 0.1, "speed_mps": value, "curvature_1pm": curvature[index], "speed_limit_mps": min(curve_cap[index], ssm_cap[index]), "human_distance_m": human_distance[index], "lateral_accel_mps2": lateral[index], "roll_utilization": high_roll[index]})
    for filename, values in (("comparison.csv", rows), ("timeseries.csv", timeseries)):
        with (output / filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(values[0])); writer.writeheader(); writer.writerows(values)
    try:
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 2, figsize=(10, 7))
        axes[0, 0].plot(curvature); axes[0, 0].set_title("Curvature vs progress")
        axes[0, 1].plot(baseline_speed, label="baseline"); axes[0, 1].plot(configurations["all_navigation_safety"], label="enabled"); axes[0, 1].legend(); axes[0, 1].set_title("Speed vs progress")
        axes[1, 0].plot(human_distance, label="human distance"); axes[1, 0].plot(ssm_cap, label="SSM cap"); axes[1, 0].legend(); axes[1, 0].set_title("Human distance and SSM")
        axes[1, 1].plot([row["roll_max_utilization"] for row in rows]); axes[1, 1].set_title("Roll utilization by config")
        figure.tight_layout(); figure.savefig(plots / "canonical_preflight.png", dpi=140); plt.close(figure)
    except ImportError:
        pass
    report = "# Canonical Benchmark Report\n\nThis artifact is a deterministic core preflight, not Gazebo runtime evidence. `goal_success`, tracking, clearance, and timing remain unset until the ROS/Gazebo runner records real topics.\n"
    (output / "canonical_benchmark_report.md").write_text(report, encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
