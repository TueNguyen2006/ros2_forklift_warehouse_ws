"""Footprint transformation and linear corridor feasibility checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from warehouse_visual_localization.core.types import Pose2D


@dataclass(frozen=True)
class CorridorBoundary:
    normal: tuple[float, float]
    bound: float


@dataclass(frozen=True)
class CorridorResult:
    transformed_footprint: np.ndarray
    left_violation: np.ndarray
    right_violation: np.ndarray
    corner_violation: np.ndarray
    maximum_violation: float
    minimum_margin: float
    feasible: bool


def transform_footprint(body_points: Sequence[Sequence[float]], pose: Pose2D) -> np.ndarray:
    """Apply ``p_world = [x,y]^T + R(yaw) p_body`` to an arbitrary polygon.

    Units are metres and radians. The body-frame origin may be offset from the
    geometric footprint centre; callers provide the actual YAML-defined points.
    """
    points = np.asarray(body_points, dtype=float)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("body_points must be an Nx2 polygon with at least three points")
    values = np.array([pose.x, pose.y, pose.yaw], dtype=float)
    if not np.all(np.isfinite(values)) or not np.all(np.isfinite(points)):
        raise ValueError("pose and footprint values must be finite")
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    rotation = np.array([[c, -s], [s, c]], dtype=float)
    return points @ rotation.T + np.array([pose.x, pose.y])


def evaluate_corridor(
    body_points: Sequence[Sequence[float]],
    pose: Pose2D,
    left: CorridorBoundary,
    right: CorridorBoundary,
    tolerance: float = 1.0e-9,
) -> CorridorResult:
    """Check ``n_L^T p <= b_L`` and ``n_R^T p >= b_R`` for every vertex.

    Positive violation means outside the corridor. Margins are positive inside
    and are expressed in the un-normalized boundary coordinate; use normalized
    normals for a margin in metres.
    """
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("tolerance must be finite and non-negative")
    footprint = transform_footprint(body_points, pose)
    left_n = np.asarray(left.normal, dtype=float).reshape(2)
    right_n = np.asarray(right.normal, dtype=float).reshape(2)
    bounds = np.array([left.bound, right.bound], dtype=float)
    if not np.all(np.isfinite(left_n)) or not np.all(np.isfinite(right_n)) or not np.all(np.isfinite(bounds)):
        raise ValueError("corridor boundaries must be finite")
    if np.linalg.norm(left_n) <= 1.0e-12 or np.linalg.norm(right_n) <= 1.0e-12:
        raise ValueError("corridor normals must be non-zero")
    left_margin = left.bound - footprint @ left_n
    right_margin = footprint @ right_n - right.bound
    left_violation = np.maximum(0.0, -left_margin)
    right_violation = np.maximum(0.0, -right_margin)
    corner_violation = np.maximum(left_violation, right_violation)
    margins = np.minimum(left_margin / np.linalg.norm(left_n), right_margin / np.linalg.norm(right_n))
    maximum = float(np.max(corner_violation))
    return CorridorResult(footprint, left_violation, right_violation, corner_violation, maximum, float(np.min(margins)), maximum <= tolerance)

