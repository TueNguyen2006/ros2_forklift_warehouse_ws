import math
from dataclasses import dataclass

from warehouse_visual_localization.core.math_utils import clamp_abs
from warehouse_visual_localization.core.types import ControlCommand, VehicleParams, VehicleState


@dataclass
class CBFSafetyConfig:
    min_obstacle_distance: float = 0.85
    cbf_gain: float = 1.4
    max_lateral_accel: float = 0.65
    max_decel: float = 0.65


class CBFSafetyLayer:
    """Simple control barrier layer for standalone validation."""

    def __init__(self, config: CBFSafetyConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        self.config = config or CBFSafetyConfig()
        self.vehicle = vehicle or VehicleParams()

    def filter(self, state: VehicleState, command: ControlCommand, obstacle_distance: float) -> ControlCommand:
        cfg = self.config
        safe_command = ControlCommand(v=command.v, yaw_rate=command.yaw_rate, steer=command.steer, accel=command.accel)
        barrier = obstacle_distance - cfg.min_obstacle_distance
        if barrier < 0.0:
            safe_command.v = min(safe_command.v, 0.0)
            safe_command.accel = min(safe_command.accel, -cfg.max_decel)
        else:
            max_closing_speed = cfg.cbf_gain * barrier
            safe_command.v = min(safe_command.v, max_closing_speed)

        max_yaw_for_lat = cfg.max_lateral_accel / max(abs(state.v), 0.05)
        safe_command.yaw_rate = clamp_abs(safe_command.yaw_rate, max_yaw_for_lat)
        if abs(safe_command.v) > 1e-4 and abs(safe_command.yaw_rate) > 1e-4:
            desired_r = abs(safe_command.v / safe_command.yaw_rate)
            min_r = max(self.vehicle.wheelbase / max(math.tan(abs(self.vehicle.max_steer)), 1e-3), 0.1)
            if desired_r < min_r:
                safe_command.yaw_rate = math.copysign(abs(safe_command.v) / min_r, safe_command.yaw_rate)
        return safe_command


@dataclass
class HalfPlane:
    ax: float
    ay: float
    b: float

    def h(self, x: float, y: float) -> float:
        return self.b - self.ax * x - self.ay * y


class PolygonCBF2D:
    """Velocity projection for a convex safe polygon represented by half-planes.

    Safe set: a_i * x + a_j * y <= b for every half-plane.
    CBF condition for single-integrator position dynamics:
    a_i * u <= gamma * h_i(x).
    """

    def __init__(self, halfplanes: list[HalfPlane], gamma: float = 1.8) -> None:
        self.halfplanes = halfplanes
        self.gamma = gamma

    @staticmethod
    def octagon(limit: float = 1.5, diagonal: float = 2.25, gamma: float = 1.8) -> "PolygonCBF2D":
        return PolygonCBF2D(
            [
                HalfPlane(1.0, 0.0, limit),
                HalfPlane(-1.0, 0.0, limit),
                HalfPlane(0.0, 1.0, limit),
                HalfPlane(0.0, -1.0, limit),
                HalfPlane(1.0, 1.0, diagonal),
                HalfPlane(1.0, -1.0, diagonal),
                HalfPlane(-1.0, 1.0, diagonal),
                HalfPlane(-1.0, -1.0, diagonal),
            ],
            gamma,
        )

    def min_h(self, x: float, y: float) -> float:
        return min(plane.h(x, y) for plane in self.halfplanes)

    def filter_velocity(self, x: float, y: float, ux: float, uy: float) -> tuple[float, float]:
        vx = ux
        vy = uy
        for plane in self.halfplanes:
            h = plane.h(x, y)
            lhs = plane.ax * vx + plane.ay * vy
            rhs = self.gamma * h
            if lhs > rhs:
                norm_sq = plane.ax * plane.ax + plane.ay * plane.ay
                correction = (lhs - rhs) / max(norm_sq, 1e-9)
                vx -= correction * plane.ax
                vy -= correction * plane.ay
        return vx, vy
