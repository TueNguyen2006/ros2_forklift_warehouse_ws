#!/usr/bin/env python3
"""Headless fork-tip docking tolerance scenario using the shared core API."""

import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "src" / "warehouse_visual_localization"
sys.path.insert(0, str(SOURCE))
from warehouse_visual_localization.core.docking_geometry import DockingConfig, evaluate_docking  # noqa: E402


def main() -> None:
    output = Path.home() / "ros2_forklift_warehouse_artifacts" / "canonical_benchmarks" / "docking"
    output.mkdir(parents=True, exist_ok=True)
    config = DockingConfig(insertion_length=1.0, pocket_width=0.08, clearance_margin=0.01)
    cases = [("perfect", 0.0, 0.0), ("lateral_2cm", 0.02, 0.0), ("heading_2deg", 0.0, math.radians(2.0)), ("combined", 0.02, math.radians(2.0))]
    rows = []
    for name, lateral, heading in cases:
        result = evaluate_docking(lateral, heading, config)
        rows.append({"case": name, "lateral_error_m": lateral, "heading_error_rad": heading, "tip_error_m": result.exact_tip_error, "base_pose_within_2cm": abs(lateral) <= 0.02, "docking_success": result.docking_feasible})
    with (output / "timeseries.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
