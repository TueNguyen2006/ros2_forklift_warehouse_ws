import math

from warehouse_visual_localization.core.math_utils import clamp, clamp_abs, wrap_angle
from warehouse_visual_localization.core.types import ControlCommand, Pose2D, VehicleForces, VehicleParams, VehicleState


class FourWheelForkliftModel:
    """Lightweight 2D single-track approximation of a four-wheel forklift.

    The model keeps explicit four-wheel geometry for visualization and force
    checks, while using a bicycle-model state update for fast standalone tests.
    """

    def __init__(self, params: VehicleParams | None = None) -> None:
        self.params = params or VehicleParams()

    def clamp_command(self, command: ControlCommand) -> ControlCommand:
        p = self.params
        v = clamp(command.v, -abs(p.max_reverse_speed), abs(p.max_speed))
        accel_limit = p.max_accel if command.accel >= 0.0 else p.max_decel
        accel = clamp_abs(command.accel, accel_limit)
        steer = clamp(command.steer, -abs(p.max_steer), abs(p.max_steer))
        return ControlCommand(v=v, yaw_rate=command.yaw_rate, steer=steer, accel=accel)

    def yaw_rate_from_steer(self, speed: float, steer: float) -> float:
        if abs(steer) < 1e-5 or abs(speed) < 1e-5:
            return 0.0
        sign = -1.0 if self.params.steer_axle == "rear" else 1.0
        return sign * speed * math.tan(steer) / max(self.params.wheelbase, 1e-6)

    def steer_from_yaw_rate(self, speed: float, yaw_rate: float) -> float:
        if abs(speed) < 1e-5:
            return 0.0
        sign = -1.0 if self.params.steer_axle == "rear" else 1.0
        steer = math.atan(sign * yaw_rate * self.params.wheelbase / speed)
        return clamp(steer, -abs(self.params.max_steer), abs(self.params.max_steer))

    def step(self, state: VehicleState, command: ControlCommand, dt: float) -> VehicleState:
        p = self.params
        cmd = self.clamp_command(command)
        steer_delta = clamp_abs(cmd.steer - state.steer, abs(p.max_steer_rate) * dt)
        steer = clamp(state.steer + steer_delta, -abs(p.max_steer), abs(p.max_steer))

        accel = clamp_abs(cmd.v - state.v, (p.max_accel if cmd.v >= state.v else p.max_decel) * dt) / max(dt, 1e-6)
        v = clamp(
            state.v + accel * dt,
            -abs(p.max_reverse_speed),
            abs(p.max_speed),
        )
        yaw_rate = self.yaw_rate_from_steer(v, steer)
        yaw = wrap_angle(state.pose.yaw + yaw_rate * dt)
        x = state.pose.x + v * math.cos(yaw) * dt
        y = state.pose.y + v * math.sin(yaw) * dt
        return VehicleState(Pose2D(x, y, yaw), v=v, yaw_rate=yaw_rate, steer=steer, ax=accel, slip_ratio=state.slip_ratio)

    def estimate_forces(self, state: VehicleState) -> VehicleForces:
        p = self.params
        lateral_accel = state.v * state.yaw_rate
        longitudinal = p.mass * state.ax
        lateral = p.mass * lateral_accel
        front_static = p.mass * p.gravity * (p.wheelbase - p.cg_to_front_axle) / p.wheelbase
        rear_static = p.mass * p.gravity - front_static
        load_transfer = p.mass * state.ax * p.cg_height / max(p.wheelbase, 1e-6)
        normal_front = max(0.0, front_static - load_transfer)
        normal_rear = max(0.0, rear_static + load_transfer)
        yaw_moment = p.yaw_inertia * state.yaw_rate
        return VehicleForces(longitudinal, lateral, normal_front, normal_rear, yaw_moment)

    def footprint(self, state: VehicleState, length: float = 1.45, width: float = 1.0) -> list[tuple[float, float]]:
        rear = -1.10
        front = rear + length
        corners = [(front, width / 2.0), (front, -width / 2.0), (rear, -width / 2.0), (rear, width / 2.0)]
        c = math.cos(state.pose.yaw)
        s = math.sin(state.pose.yaw)
        return [
            (state.pose.x + c * x - s * y, state.pose.y + s * x + c * y)
            for x, y in corners
        ]

