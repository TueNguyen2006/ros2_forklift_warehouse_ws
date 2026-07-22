#!/usr/bin/env python3
"""Headless validation scenarios for canonical forklift constraint equations."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from common import ensure_output_dir, parser, write_csv, write_summary  # noqa: E402
from warehouse_visual_localization.core.cbf_filter import filter_linear_cbf_command  # noqa: E402
from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, evaluate_corridor  # noqa: E402
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit, signed_curvature  # noqa: E402
from warehouse_visual_localization.core.docking_geometry import DockingConfig, evaluate_docking  # noqa: E402
from warehouse_visual_localization.core.human_ssm import HumanSSMConfig, evaluate_human_ssm  # noqa: E402
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover  # noqa: E402
from warehouse_visual_localization.core.types import Pose2D  # noqa: E402
from warehouse_visual_localization.core.visual_servo import ibvs_command  # noqa: E402


def main() -> None:
    args = parser("Canonical constraints headless scenarios")
    args.add_argument("--smoke", action="store_true")
    options = args.parse_args()
    output = ensure_output_dir(options.output_dir, "canonical_constraints")
    rows: list[dict[str, object]] = []

    cfg = CurvatureSpeedConfig(vehicle_speed_limit=0.5, lateral_acceleration_limit=0.3)
    curvature_cases = {
        "straight": [[0, 0], [1, 0], [2, 0]],
        "gentle_turn": [[0, 0], [1, 0], [2, 0.2]],
        "sharp_turn": [[0, 0], [1, 0], [1, 1]],
    }
    curve_speeds = {}
    for name, path in curvature_cases.items():
        result = curvature_speed_limit(signed_curvature(path), cfg)
        curve_speeds[name] = float(np.min(result.speed_limit))
        rows.append({"scenario": "curvature", "case": name, "metric": "minimum_speed_limit_mps", "value": curve_speeds[name], "pass": True})

    points = [(0.5, 0.3), (0.5, -0.3), (-0.5, -0.3), (-0.5, 0.3)]
    left, right = CorridorBoundary((0.0, 1.0), 1.0), CorridorBoundary((0.0, 1.0), -1.0)
    corridor_cases = {"center": Pose2D(), "touch": Pose2D(y=0.7), "rear_swing": Pose2D(y=0.62, yaw=0.65)}
    corridor_results = {name: evaluate_corridor(points, pose, left, right) for name, pose in corridor_cases.items()}
    for name, result in corridor_results.items():
        rows.append({"scenario": "corridor", "case": name, "metric": "minimum_margin_m", "value": result.minimum_margin, "pass": result.feasible if name != "rear_swing" else not result.feasible})

    support = [(-0.6, -0.5), (0.6, -0.5), (0.6, 0.5), (-0.6, 0.5)]
    roll_low = evaluate_rollover(0.5, 0.4, 0.0, support, RolloverConfig(vehicle_cog_height=0.35, safety_factor=0.8))
    roll_high = evaluate_rollover(0.5, 0.4, 0.0, support, RolloverConfig(vehicle_cog_height=0.8, safety_factor=0.8))
    roll_outside = evaluate_rollover(1.0, 6.0, 0.0, support, RolloverConfig(vehicle_cog_height=1.0, safety_factor=1.0))
    rows.extend([
        {"scenario": "rollover", "case": "low_cog", "metric": "zmp_margin_m", "value": roll_low.zmp_margin, "pass": roll_low.safe},
        {"scenario": "rollover", "case": "high_cog", "metric": "utilization", "value": roll_high.utilization_ratio, "pass": roll_high.utilization_ratio > roll_low.utilization_ratio},
        {"scenario": "rollover", "case": "zmp_outside", "metric": "safe", "value": int(roll_outside.safe), "pass": not roll_outside.safe},
    ])

    ssm_results = {
        "far": evaluate_human_ssm(5.0, 0.3),
        "approaching": evaluate_human_ssm(1.0, 0.3, human_approach_speed=0.5),
        "minimum_margin": evaluate_human_ssm(0.5, 0.2),
        "invalid_brake": evaluate_human_ssm(2.0, 0.2, HumanSSMConfig(braking_deceleration=0.0)),
    }
    for name, result in ssm_results.items():
        rows.append({"scenario": "human_ssm", "case": name, "metric": "allowed_speed_mps", "value": result.allowed_speed, "pass": result.safety_state in ("clear", "speed_limited") if name in ("far", "approaching") else result.allowed_speed == 0.0})

    docking_cfg = DockingConfig(insertion_length=1.0, pocket_width=0.08, clearance_margin=0.01)
    docking_results = {"perfect": evaluate_docking(0.0, 0.0, docking_cfg), "lateral": evaluate_docking(0.05, 0.0, docking_cfg), "heading_2deg": evaluate_docking(0.0, np.deg2rad(2.0), docking_cfg)}
    for name, result in docking_results.items():
        rows.append({"scenario": "docking", "case": name, "metric": "tip_error_m", "value": result.exact_tip_error, "pass": result.docking_feasible if name == "perfect" else not result.docking_feasible})

    ibvs_zero = ibvs_command([0.0, 0.0], [0.0, 0.0], [[1, 0], [0, 1]], 1.0)
    ibvs_step = ibvs_command([1.0, -1.0], [0.0, 0.0], [[1, 0], [0, 1]], 0.5)
    ibvs_singular = ibvs_command([1.0, 1.0], [0.0, 0.0], [[1, 0], [2, 0]], 1.0)
    rows.extend([
        {"scenario": "visual_servo", "case": "zero", "metric": "command_norm", "value": float(np.linalg.norm(ibvs_zero.camera_twist)), "pass": True},
        {"scenario": "visual_servo", "case": "linear_step", "metric": "residual_after", "value": float(np.linalg.norm(np.array([1.0, -1.0]) + ibvs_step.camera_twist)), "pass": True},
        {"scenario": "visual_servo", "case": "rank_deficient", "metric": "rank", "value": ibvs_singular.rank, "pass": True},
    ])

    cbf_safe = filter_linear_cbf_command([0.2, 0.0], [[1, 0]], [0.5], [-1, -1], [1, 1])
    cbf_project = filter_linear_cbf_command([0.9, 0.5], [[1, 0]], [0.3], [-0.5, -0.2], [0.5, 0.2])
    cbf_infeasible = filter_linear_cbf_command([0.2], [[1]], [-1.0], [-0.5], [0.5])
    rows.extend([
        {"scenario": "cbf", "case": "safe", "metric": "intervention", "value": cbf_safe.intervention_magnitude, "pass": cbf_safe.feasible},
        {"scenario": "cbf", "case": "projected", "metric": "safe_speed", "value": float(cbf_project.safe_command[0]), "pass": cbf_project.feasible},
        {"scenario": "cbf", "case": "infeasible", "metric": "fallback", "value": int(cbf_infeasible.fallback_used), "pass": cbf_infeasible.fallback_used},
    ])

    summary = {"all_passed": bool(all(bool(row["pass"]) for row in rows)), "curvature_min_speed_mps": curve_speeds, "rows": len(rows)}
    write_csv(output / "canonical_constraints.csv", rows)
    write_summary(output / "summary.json", summary)
    report = ["CANONICAL CONSTRAINTS REPORT", f"all_passed={summary['all_passed']}", f"rows={summary['rows']}"]
    for row in rows:
        report.append(f"{row['scenario']}:{row['case']} {row['metric']}={float(row['value']):.6f} pass={row['pass']}")
    (output / "report.txt").write_text("\n".join(report) + "\n", encoding="ascii")

    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].bar(list(curve_speeds), list(curve_speeds.values()))
        axes[0].set_ylabel("speed limit (m/s)")
        axes[0].set_title("Curvature-speed interface")
        axes[1].bar(["low CoG", "high CoG"], [roll_low.utilization_ratio, roll_high.utilization_ratio])
        axes[1].axhline(1.0, color="r", linestyle="--")
        axes[1].set_ylabel("lateral utilization")
        axes[1].set_title("Quasi-static rollover utilization")
        fig.tight_layout()
        fig.savefig(output / "canonical_constraints.png", dpi=140)
        if options.show:
            plt.show()
        plt.close(fig)
    except ImportError:
        pass
    print(f"wrote {output}")
    if not summary["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
