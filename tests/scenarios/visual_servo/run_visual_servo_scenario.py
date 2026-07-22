#!/usr/bin/env python3
"""Synthetic headless IBVS/PBVS micro-control convergence scenario."""

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "warehouse_visual_localization"))
from warehouse_visual_localization.core.visual_servo import ibvs_command, planar_pbvs_command  # noqa: E402


def main() -> None:
    output = Path.home() / "ros2_forklift_warehouse_artifacts" / "canonical_benchmarks" / "visual_servo"
    output.mkdir(parents=True, exist_ok=True)
    feature = np.array([1.0, -0.8])
    rows = []
    for step in range(30):
        result = ibvs_command(feature, [0.0, 0.0], [[1.0, 0.0], [0.0, 1.0]], 0.25, output_limits=[0.3, 0.3])
        feature = feature + result.camera_twist * 0.2
        rows.append({"step": step, "feature_error": float(np.linalg.norm(feature)), "command_norm": float(np.linalg.norm(result.camera_twist))})
    pbvs = planar_pbvs_command([0.1, -0.1, 0.2], [0.0, 0.0, 0.0])
    with (output / "timeseries.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps({"initial_feature_error": 1.280624847, "final_feature_error": rows[-1]["feature_error"], "convergence_steps": len(rows), "pbvs_command": pbvs.command.tolist()}, indent=2), encoding="utf-8")
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
