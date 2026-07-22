"""Conservative quasi-static lateral acceleration and ZMP safety checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class RolloverConfig:
    gravity: float = 9.81
    track_width: float = 1.0
    vehicle_mass: float = 1000.0
    vehicle_cog: tuple[float, float] = (0.0, 0.0)
    vehicle_cog_height: float = 0.5
    safety_factor: float = 0.5


@dataclass(frozen=True)
class RolloverResult:
    lateral_acceleration: float
    lateral_acceleration_limit: float
    utilization_ratio: float
    zmp: np.ndarray
    zmp_margin: float
    safe: bool
    reason_code: str
    total_cog: np.ndarray


def combined_center_of_gravity(
    vehicle_mass: float,
    vehicle_cog: Sequence[float],
    load_mass: float = 0.0,
    load_cog: Sequence[float] = (0.0, 0.0),
) -> np.ndarray:
    """Compute ``(m_v p_v + m_l p_l) / (m_v + m_l)`` in metres."""
    masses = np.asarray([vehicle_mass, load_mass], dtype=float)
    vehicle = np.asarray(vehicle_cog, dtype=float).reshape(2)
    load = np.asarray(load_cog, dtype=float).reshape(2)
    if not np.all(np.isfinite(masses)) or not np.all(np.isfinite(vehicle)) or not np.all(np.isfinite(load)) or vehicle_mass <= 0.0 or load_mass < 0.0:
        raise ValueError("masses and CoG values must be finite; vehicle mass must be positive")
    return (vehicle_mass * vehicle + load_mass * load) / (vehicle_mass + load_mass)


def point_in_convex_polygon(point: Sequence[float], polygon: Sequence[Sequence[float]], tolerance: float = 1.0e-9) -> bool:
    """Return whether a point lies in/on a convex polygon with either winding."""
    point_value = np.asarray(point, dtype=float).reshape(2)
    vertices = np.asarray(polygon, dtype=float)
    if vertices.ndim != 2 or vertices.shape[0] < 3 or vertices.shape[1] != 2 or not np.all(np.isfinite(vertices)) or not np.all(np.isfinite(point_value)):
        raise ValueError("point and polygon must be finite 2D values")
    signs: list[float] = []
    for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
        edge = end - start
        relative = point_value - start
        cross = edge[0] * relative[1] - edge[1] * relative[0]
        if abs(cross) > tolerance:
            signs.append(float(np.sign(cross)))
    return not signs or all(sign >= 0.0 for sign in signs) or all(sign <= 0.0 for sign in signs)


def convex_polygon_margin(point: Sequence[float], polygon: Sequence[Sequence[float]]) -> float:
    """Signed minimum edge distance; positive inside a consistently convex polygon."""
    point_value = np.asarray(point, dtype=float).reshape(2)
    vertices = np.asarray(polygon, dtype=float)
    crosses = []
    distances = []
    for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
        edge = end - start
        cross = edge[0] * (point_value[1] - start[1]) - edge[1] * (point_value[0] - start[0])
        crosses.append(cross)
        distances.append(abs(cross) / max(float(np.linalg.norm(edge)), 1.0e-12))
    inside = all(value >= -1.0e-9 for value in crosses) or all(value <= 1.0e-9 for value in crosses)
    return float(min(distances) if inside else -min(distances))


def evaluate_rollover(
    speed: float,
    curvature: float,
    longitudinal_acceleration: float,
    support_polygon: Sequence[Sequence[float]],
    config: RolloverConfig | None = None,
    load_mass: float = 0.0,
    load_cog: Sequence[float] = (0.0, 0.0),
    combined_cog_height: float | None = None,
    floor_grade: float = 0.0,
) -> RolloverResult:
    """Evaluate a quasi-static ZMP approximation, not a full rollover model.

    ``a_y=v^2*kappa``, ``p_zmp=p_cog+h/g*[a_x,a_y]`` and
    ``|a_y| <= T*g*cos(theta)/(2*h)``. It excludes suspension, tire dynamics,
    mast compliance, and a counterbalance forklift's rear-pivot stability triangle.
    """
    cfg = config or RolloverConfig()
    values = np.asarray([speed, curvature, longitudinal_acceleration, cfg.gravity, cfg.track_width, cfg.vehicle_cog_height, cfg.safety_factor, floor_grade], dtype=float)
    if not np.all(np.isfinite(values)) or cfg.gravity <= 0.0 or cfg.track_width <= 0.0 or cfg.vehicle_cog_height <= 0.0 or cfg.safety_factor <= 0.0:
        raise ValueError("rollover inputs must be finite and physical limits positive")
    height = float(combined_cog_height if combined_cog_height is not None else cfg.vehicle_cog_height)
    if not np.isfinite(height) or height <= 0.0:
        raise ValueError("combined_cog_height must be finite and positive")
    cog = combined_center_of_gravity(cfg.vehicle_mass, cfg.vehicle_cog, load_mass, load_cog)
    lateral = float(speed * speed * curvature)
    raw_limit = cfg.track_width * cfg.gravity * max(np.cos(floor_grade), 0.0) / (2.0 * height)
    limit = cfg.safety_factor * raw_limit
    zmp = cog + height / cfg.gravity * np.array([longitudinal_acceleration, lateral])
    margin = convex_polygon_margin(zmp, support_polygon)
    utilization = abs(lateral) / max(limit, 1.0e-12)
    safe = bool(utilization <= 1.0 + 1.0e-9 and point_in_convex_polygon(zmp, support_polygon))
    reason = "safe" if safe else ("lateral_acceleration_limit" if utilization > 1.0 else "zmp_outside_support")
    return RolloverResult(lateral, limit, utilization, zmp, margin, safe, reason, cog)

