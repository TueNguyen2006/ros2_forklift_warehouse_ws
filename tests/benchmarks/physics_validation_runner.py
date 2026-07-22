#!/usr/bin/env python3
"""Aggregate real Gazebo logger CSVs without inventing physics metrics."""
from __future__ import annotations
import csv, json
from pathlib import Path

ARTIFACT_ROOT = Path.home() / "ros2_forklift_warehouse_artifacts" / "canonical_benchmarks"

def main() -> None:
    source = ARTIFACT_ROOT / "gazebo_latest" / "timeseries.csv"
    output = ARTIFACT_ROOT / "physics_validation"
    output.mkdir(parents=True, exist_ok=True)
    rows = list(csv.DictReader(source.open(encoding="utf-8"))) if source.exists() else []
    summary = {"source": str(source), "samples": len(rows), "status": "recorded" if rows else "no_runtime_samples"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (output / "physics_validation_report.md").write_text("# Physics Validation Report\n\n" + ("Runtime samples recorded. Contact metrics require a contact-sensor plugin." if rows else "No Gazebo runtime samples recorded; no safety claim is made."), encoding="utf-8")
    print(f"wrote {output}")

if __name__ == "__main__": main()
