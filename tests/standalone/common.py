#!/usr/bin/env python3
import argparse
import csv
import json
import sys
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKSPACE_ROOT / "src" / "warehouse_visual_localization"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from warehouse_visual_localization.core import (  # noqa: E402
    CBFSafetyConfig,
    FrictionConfig,
    PurePursuitConfig,
    VehicleParams,
    VelocityProfileConfig,
)
from warehouse_visual_localization.core.localization import EKF2DConfig  # noqa: E402


DEFAULT_CONFIG = SRC_ROOT / "config" / "core" / "forklift_2d.yaml"
DEFAULT_OUTPUT = Path.home() / "ros2_forklift_warehouse_artifacts" / "standalone_tests"


def parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    p.add_argument("--show", action="store_true")
    p.add_argument("--dt", type=float, default=0.05)
    return p


def _dataclass_from_dict(cls, values: dict[str, Any]):
    allowed = {field.name for field in fields(cls)}
    return cls(**{key: value for key, value in values.items() if key in allowed})


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_vehicle_config(path: str | Path) -> tuple[VehicleParams, PurePursuitConfig, CBFSafetyConfig, FrictionConfig, VelocityProfileConfig, EKF2DConfig]:
    cfg = load_config(path)
    return (
        _dataclass_from_dict(VehicleParams, cfg.get("vehicle", {})),
        _dataclass_from_dict(PurePursuitConfig, cfg.get("controller", {})),
        _dataclass_from_dict(CBFSafetyConfig, cfg.get("safety", {})),
        _dataclass_from_dict(FrictionConfig, cfg.get("friction", {})),
        _dataclass_from_dict(VelocityProfileConfig, cfg.get("velocity_profile", {})),
        _dataclass_from_dict(EKF2DConfig, cfg.get("localization", {})),
    )


def ensure_output_dir(output_dir: str | Path, name: str) -> Path:
    path = Path(output_dir).expanduser().resolve() / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8")


def dataclass_dict(item: Any) -> dict[str, Any]:
    return asdict(item)

