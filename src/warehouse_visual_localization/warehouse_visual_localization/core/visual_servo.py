"""Independent IBVS and planar PBVS micro-control laws."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from warehouse_visual_localization.core.math_utils import wrap_angle


@dataclass(frozen=True)
class IBVSResult:
    camera_twist: np.ndarray
    residual_norm: float
    rank: int
    condition_number: float
    valid: bool
    reason: str


@dataclass(frozen=True)
class PBVSResult:
    command: np.ndarray
    error: np.ndarray


def ibvs_command(
    current_features: Sequence[float],
    desired_features: Sequence[float],
    interaction_matrix: Sequence[Sequence[float]],
    gain: float,
    pseudo_inverse_tolerance: float = 1.0e-6,
    output_limits: Sequence[float] | None = None,
) -> IBVSResult:
    """Return ``v_c=-lambda*pinv(L_s)*(s-s*)`` with finite/rank diagnostics.

    Inputs are unitless image features; the interaction matrix maps camera twist
    to feature rate. This does not estimate depth or calibrate a camera.
    """
    current = np.asarray(current_features, dtype=float).reshape(-1)
    desired = np.asarray(desired_features, dtype=float).reshape(-1)
    matrix = np.asarray(interaction_matrix, dtype=float)
    if current.shape != desired.shape or matrix.ndim != 2 or matrix.shape[0] != current.size or not np.all(np.isfinite(current)) or not np.all(np.isfinite(desired)) or not np.all(np.isfinite(matrix)) or not math.isfinite(gain) or gain < 0.0 or not math.isfinite(pseudo_inverse_tolerance) or pseudo_inverse_tolerance <= 0.0:
        width = matrix.shape[1] if matrix.ndim == 2 else 0
        return IBVSResult(np.zeros(width), math.inf, 0, math.inf, False, "invalid_input")
    rank = int(np.linalg.matrix_rank(matrix, tol=pseudo_inverse_tolerance))
    try:
        condition = float(np.linalg.cond(matrix)) if min(matrix.shape) else math.inf
        command = -gain * (np.linalg.pinv(matrix, rcond=pseudo_inverse_tolerance) @ (current - desired))
    except np.linalg.LinAlgError:
        return IBVSResult(np.zeros(matrix.shape[1]), math.inf, rank, math.inf, False, "linear_algebra_failure")
    if output_limits is not None:
        limits = np.asarray(output_limits, dtype=float).reshape(-1)
        if limits.size != command.size or not np.all(np.isfinite(limits)) or np.any(limits < 0.0):
            return IBVSResult(np.zeros(matrix.shape[1]), math.inf, rank, condition, False, "invalid_limits")
        command = np.clip(command, -limits, limits)
    valid = bool(np.all(np.isfinite(command)) and rank > 0)
    return IBVSResult(np.nan_to_num(command), float(np.linalg.norm(current - desired)), rank, condition, valid, "ok" if valid else "rank_deficient")


def planar_pbvs_command(current_pose: Sequence[float], desired_pose: Sequence[float], gains: Sequence[float] = (1.0, 1.0, 1.0), output_limits: Sequence[float] | None = None) -> PBVSResult:
    """Planar PBVS adapter ``u=-K[e_x,e_y,wrap(e_psi)]`` for forklift docking.

    Pose units are metres/metres/radians and command units follow the chosen
    gains. It is not a full SE(3) logarithm or six-DoF visual-servo controller.
    """
    current = np.asarray(current_pose, dtype=float).reshape(-1)
    desired = np.asarray(desired_pose, dtype=float).reshape(-1)
    gain_values = np.asarray(gains, dtype=float).reshape(-1)
    if current.size != 3 or desired.size != 3 or gain_values.size != 3 or not np.all(np.isfinite(current)) or not np.all(np.isfinite(desired)) or not np.all(np.isfinite(gain_values)) or np.any(gain_values < 0.0):
        raise ValueError("planar PBVS expects finite [x, y, yaw] poses and non-negative gains")
    error = current - desired
    error[2] = wrap_angle(float(error[2]))
    command = -gain_values * error
    if output_limits is not None:
        limits = np.asarray(output_limits, dtype=float).reshape(-1)
        if limits.size != 3 or not np.all(np.isfinite(limits)) or np.any(limits < 0.0):
            raise ValueError("output_limits must be three finite non-negative values")
        command = np.clip(command, -limits, limits)
    return PBVSResult(command, error)

