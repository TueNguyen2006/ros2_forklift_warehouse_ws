"""Small bounded projection solver for linear planar CBF command constraints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class CBFProjectionResult:
    safe_command: np.ndarray
    intervention_magnitude: float
    active_constraints: tuple[int, ...]
    feasible: bool
    fallback_used: bool


def filter_linear_cbf_command(
    desired_command: Sequence[float],
    a_matrix: Sequence[Sequence[float]] | np.ndarray,
    b_vector: Sequence[float] | np.ndarray,
    lower_bounds: Sequence[float],
    upper_bounds: Sequence[float],
    tolerance: float = 1.0e-8,
    maximum_iterations: int = 32,
) -> CBFProjectionResult:
    """Project command onto ``A u <= b`` and box bounds using cyclic halfspaces.

    This is a bounded active-set-like projection for *linear* control-space
    constraints, not a general nonlinear QP solver. Invalid/infeasible inputs or
    non-convergence fail safe to the zero command clipped to actuator bounds.
    """
    desired = np.asarray(desired_command, dtype=float).reshape(-1)
    lower = np.asarray(lower_bounds, dtype=float).reshape(-1)
    upper = np.asarray(upper_bounds, dtype=float).reshape(-1)
    matrix = np.asarray(a_matrix, dtype=float)
    bound = np.asarray(b_vector, dtype=float).reshape(-1)
    dimension = desired.size
    zero = np.zeros(dimension, dtype=float)
    if dimension == 0 or lower.size != dimension or upper.size != dimension or matrix.ndim != 2 or matrix.shape[1] != dimension or matrix.shape[0] != bound.size or not np.all(np.isfinite(desired)) or not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(bound)) or np.any(lower > upper) or tolerance <= 0.0 or maximum_iterations <= 0:
        return CBFProjectionResult(np.clip(zero, np.minimum(lower, upper) if lower.size == dimension else 0.0, np.maximum(lower, upper) if upper.size == dimension else 0.0), 0.0, (), False, True)
    command = np.clip(desired, lower, upper)
    for _ in range(int(maximum_iterations)):
        previous = command.copy()
        for row, rhs in zip(matrix, bound):
            excess = float(row @ command - rhs)
            norm_squared = float(row @ row)
            if excess > tolerance:
                if norm_squared <= tolerance:
                    return CBFProjectionResult(np.clip(zero, lower, upper), float(np.linalg.norm(desired)), (), False, True)
                command = command - excess / norm_squared * row
                command = np.clip(command, lower, upper)
        if float(np.linalg.norm(command - previous)) <= tolerance:
            break
    violations = matrix @ command - bound
    if np.any(violations > tolerance):
        return CBFProjectionResult(np.clip(zero, lower, upper), float(np.linalg.norm(desired)), (), False, True)
    active = tuple(int(index) for index, value in enumerate(violations) if abs(float(value)) <= max(tolerance * 10.0, 1.0e-7))
    return CBFProjectionResult(command, float(np.linalg.norm(command - desired)), active, True, False)

