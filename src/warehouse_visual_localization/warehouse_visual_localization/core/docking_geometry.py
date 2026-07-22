"""Planar fork-tip docking tolerance calculations."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DockingConfig:
    insertion_length: float = 1.0
    pocket_width: float = 0.20
    clearance_margin: float = 0.02


@dataclass(frozen=True)
class DockingResult:
    approximate_tip_error: float
    exact_tip_error: float
    maximum_heading_error: float
    lateral_feasible: bool
    angular_feasible: bool
    docking_feasible: bool
    failure_reason: str


def evaluate_docking(error_y: float, error_yaw: float, config: DockingConfig | None = None) -> DockingResult:
    """Check fork-tip tolerance using small-angle and exact planar geometry.

    ``e_tip ~= e_y + l e_psi`` and ``e_tip_exact=e_y+l*sin(e_psi)``. Errors
    are metres/radians. The model assumes a rigid planar fork and does not model
    pocket depth, mast flex, pallet deformation, or perception uncertainty.
    """
    cfg = config or DockingConfig()
    values = [error_y, error_yaw, cfg.insertion_length, cfg.pocket_width, cfg.clearance_margin]
    if not all(math.isfinite(float(item)) for item in values) or cfg.insertion_length <= 0.0 or cfg.pocket_width <= 0.0 or cfg.clearance_margin < 0.0:
        raise ValueError("docking values must be finite; length/width positive and margin non-negative")
    half_clearance = cfg.pocket_width / 2.0 - cfg.clearance_margin
    if half_clearance <= 0.0:
        raise ValueError("pocket_width must exceed twice clearance_margin")
    approximate = error_y + cfg.insertion_length * error_yaw
    exact = error_y + cfg.insertion_length * math.sin(error_yaw)
    heading_max = half_clearance / cfg.insertion_length
    lateral_ok = abs(exact) <= half_clearance + 1.0e-12
    angular_ok = abs(error_yaw) <= heading_max + 1.0e-12
    feasible = lateral_ok and angular_ok
    reason = "safe" if feasible else ("tip_lateral_error" if not lateral_ok else "heading_error")
    return DockingResult(approximate, exact, heading_max, lateral_ok, angular_ok, feasible, reason)

