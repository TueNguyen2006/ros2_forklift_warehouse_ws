"""Curvature estimation and curvature-limited reference speed scheduling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class CurvatureSpeedConfig:
    """Parameters for a geometric speed limit.

    Formula: ``v_max = min(v_bar, sqrt(a_y_max / (abs(kappa) + epsilon)))``.
    Curvature is in 1/m, speed is in m/s, and lateral acceleration is in m/s^2.
    This quasi-static interface does not model tire slip or transient dynamics.
    """

    vehicle_speed_limit: float = 0.32
    lateral_acceleration_limit: float = 0.30
    epsilon: float = 1.0e-6
    smoothing_window: int = 0


@dataclass(frozen=True)
class CurvatureSpeedResult:
    curvature: np.ndarray
    speed_limit: np.ndarray
    effective_lateral_acceleration_limit: float


def _finite_positive(value: float, name: str, allow_zero: bool = False) -> float:
    value = float(value)
    if not np.isfinite(value) or (value < 0.0 if allow_zero else value <= 0.0):
        raise ValueError(f"{name} must be finite and {'non-negative' if allow_zero else 'positive'}")
    return value


def path_to_xy(path: Any) -> np.ndarray:
    """Convert an Nx2-like array or duck-typed ``nav_msgs/Path`` into Nx2 floats.

    A ROS dependency is intentionally avoided: a Path only needs a ``poses``
    sequence whose entries expose ``pose.position.x`` and ``pose.position.y``.
    """
    if hasattr(path, "poses"):
        values = [(item.pose.position.x, item.pose.position.y) for item in path.poses]
        points = np.asarray(values, dtype=float)
    else:
        points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("path must be an Nx2 array or a Path-like object")
    points = points[:, :2]
    if not np.all(np.isfinite(points)):
        raise ValueError("path contains non-finite coordinates")
    return points


def signed_curvature(points: Any, smoothing_window: int = 0) -> np.ndarray:
    """Estimate signed curvature with a three-point geometric difference.

    ``kappa = 2 * cross(b-a, c-b) / (|b-a| |c-b| |c-a|)`` in 1/m.
    Duplicate points and paths shorter than three points produce zero rather
    than NaN. Optional smoothing averages the *result*, never alters the path.
    """
    xy = path_to_xy(points)
    count = len(xy)
    output = np.zeros(count, dtype=float)
    if count < 3:
        return output

    for index in range(1, count - 1):
        previous = index - 1
        following = index + 1
        while previous >= 0 and np.linalg.norm(xy[index] - xy[previous]) <= 1.0e-12:
            previous -= 1
        while following < count and np.linalg.norm(xy[following] - xy[index]) <= 1.0e-12:
            following += 1
        if previous < 0 or following >= count:
            continue
        first = xy[index] - xy[previous]
        second = xy[following] - xy[index]
        chord = xy[following] - xy[previous]
        denominator = np.linalg.norm(first) * np.linalg.norm(second) * np.linalg.norm(chord)
        if denominator > 1.0e-12:
            cross = first[0] * second[1] - first[1] * second[0]
            output[index] = 2.0 * cross / denominator

    if count > 2:
        output[0] = output[1]
        output[-1] = output[-2]
    window = int(smoothing_window)
    if window > 1:
        if window % 2 == 0:
            window += 1
        padded = np.pad(output, window // 2, mode="edge")
        output = np.convolve(padded, np.ones(window) / window, mode="valid")
    return np.nan_to_num(output, nan=0.0, posinf=0.0, neginf=0.0)


def curvature_speed_limit(
    curvature: Sequence[float] | np.ndarray,
    config: CurvatureSpeedConfig | None = None,
    rollover_lateral_acceleration_limit: float | None = None,
) -> CurvatureSpeedResult:
    """Compute per-sample speed limits, optionally capped by rollover safety.

    ``a_y_effective = min(a_y_configured, a_y_roll)`` when a positive,
    finite rollover limit is supplied. This produces a reference only; it does
    not alter MPPI, Nav2, or any runtime command.
    """
    cfg = config or CurvatureSpeedConfig()
    speed_cap = _finite_positive(cfg.vehicle_speed_limit, "vehicle_speed_limit")
    lateral_cap = _finite_positive(cfg.lateral_acceleration_limit, "lateral_acceleration_limit")
    epsilon = _finite_positive(cfg.epsilon, "epsilon")
    if rollover_lateral_acceleration_limit is not None:
        lateral_cap = min(lateral_cap, _finite_positive(rollover_lateral_acceleration_limit, "rollover_lateral_acceleration_limit"))
    kappa = np.asarray(curvature, dtype=float).reshape(-1)
    if not np.all(np.isfinite(kappa)):
        raise ValueError("curvature contains non-finite values")
    speeds = np.minimum(speed_cap, np.sqrt(lateral_cap / (np.abs(kappa) + epsilon)))
    return CurvatureSpeedResult(kappa, np.nan_to_num(speeds, nan=0.0, posinf=speed_cap), lateral_cap)


def acceleration_limited_speed_profile(
    speed_limits: Sequence[float] | np.ndarray,
    arc_length: Sequence[float] | np.ndarray,
    max_acceleration: float,
    max_deceleration: float,
) -> np.ndarray:
    """Make a feasible path speed reference from local ``v_max(s)`` limits.

    A backward braking pass enforces ``v_i^2 <= v_next^2 + 2*a_decel*ds``;
    a forward pass enforces the equivalent acceleration constraint. Units are
    m/s, m, and m/s^2. This does not alter the path or command a vehicle.
    """
    caps = np.asarray(speed_limits, dtype=float).reshape(-1)
    distance = np.asarray(arc_length, dtype=float).reshape(-1)
    if caps.size != distance.size or caps.size == 0 or not np.all(np.isfinite(caps)) or not np.all(np.isfinite(distance)):
        raise ValueError("speed_limits and arc_length must be finite equal-length arrays")
    accel = _finite_positive(max_acceleration, "max_acceleration")
    decel = _finite_positive(max_deceleration, "max_deceleration")
    if np.any(caps < 0.0) or np.any(np.diff(distance) < 0.0):
        raise ValueError("speed limits must be non-negative and arc length monotonic")
    result = caps.copy()
    for index in range(result.size - 2, -1, -1):
        ds = distance[index + 1] - distance[index]
        result[index] = min(result[index], np.sqrt(result[index + 1] ** 2 + 2.0 * decel * ds))
    for index in range(1, result.size):
        ds = distance[index] - distance[index - 1]
        result[index] = min(result[index], np.sqrt(result[index - 1] ** 2 + 2.0 * accel * ds))
    return result
