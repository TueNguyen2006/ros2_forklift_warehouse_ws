import math
from dataclasses import dataclass


@dataclass
class WheelCommand:
    speed: float = 0.0
    steer: float = 0.0


@dataclass
class WheelKinematicsResult:
    vx: float
    vy: float
    yaw_rate: float
    residual: float
    rear_steer: float = 0.0
    front_speed_left: float = 0.0
    front_speed_right: float = 0.0
    rear_speed_left: float = 0.0
    rear_speed_right: float = 0.0


class FourWheelKinematics:
    """Four-wheel planar kinematics from individual wheel speeds and angles."""

    def __init__(self, wheelbase: float = 1.25, track_width: float = 0.82) -> None:
        half_l = wheelbase / 2.0
        half_w = track_width / 2.0
        self.positions = {
            "front_left": (half_l, half_w),
            "front_right": (half_l, -half_w),
            "rear_left": (-half_l, half_w),
            "rear_right": (-half_l, -half_w),
        }

    def body_twist_from_wheels(self, commands: dict[str, WheelCommand]) -> WheelKinematicsResult:
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("numpy is required for wheel-level kinematics solving") from exc

        rows = []
        values = []
        for name, (rx, ry) in self.positions.items():
            cmd = commands[name]
            c = math.cos(cmd.steer)
            s = math.sin(cmd.steer)
            rows.append([c, s, -c * ry + s * rx])
            values.append(cmd.speed)

        a = np.array(rows, dtype=float)
        b = np.array(values, dtype=float)
        solution, residuals, _, _ = np.linalg.lstsq(a, b, rcond=None)
        predicted = a @ solution
        residual = float(np.linalg.norm(predicted - b))
        return WheelKinematicsResult(
            vx=float(solution[0]),
            vy=float(solution[1]),
            yaw_rate=float(solution[2]),
            residual=residual,
        )

    def rear_steer_twist_from_wheels(
        self,
        front_left_speed: float,
        front_right_speed: float,
        rear_left_speed: float,
        rear_right_speed: float,
        rear_steer: float,
    ) -> WheelKinematicsResult:
        """Nonholonomic four-wheel rear-steer forklift kinematics.

        Assumptions:
        - Body frame origin is at the midpoint of the fixed front axle.
        - Positive x is forward, positive y is left.
        - Front wheels are fixed and roll along body x.
        - Rear wheels steer by `rear_steer`.
        - No lateral velocity at the fixed front axle: vy_front = 0.
        - Wheel speeds are longitudinal rolling speeds at each wheel.
        """
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("numpy is required for rear-steer kinematics solving") from exc

        half_track = abs(self.positions["front_left"][1])
        wheelbase = abs(self.positions["front_left"][0] - self.positions["rear_left"][0])
        c = math.cos(rear_steer)
        s = math.sin(rear_steer)

        # Unknowns are [vx_front, yaw_rate]. The reference point is the fixed
        # front axle midpoint, so lateral velocity at that point is zero.
        rows = [
            [1.0, -half_track],   # front left
            [1.0, half_track],    # front right
            [c, -c * half_track - s * wheelbase],  # rear left
            [c, c * half_track - s * wheelbase],   # rear right
        ]
        values = [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]
        a = np.array(rows, dtype=float)
        b = np.array(values, dtype=float)
        solution, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
        predicted = a @ solution
        residual = float(np.linalg.norm(predicted - b))
        return WheelKinematicsResult(
            vx=float(solution[0]),
            vy=0.0,
            yaw_rate=float(solution[1]),
            residual=residual,
            rear_steer=rear_steer,
            front_speed_left=front_left_speed,
            front_speed_right=front_right_speed,
            rear_speed_left=rear_left_speed,
            rear_speed_right=rear_right_speed,
        )

    def ideal_rear_steer_wheel_speeds(self, vx: float, rear_steer: float) -> dict[str, float]:
        half_track = abs(self.positions["front_left"][1])
        wheelbase = abs(self.positions["front_left"][0] - self.positions["rear_left"][0])
        if abs(rear_steer) < 1e-6:
            yaw_rate = 0.0
        else:
            yaw_rate = -vx * math.tan(rear_steer) / max(wheelbase, 1e-9)
        c = math.cos(rear_steer)
        s = math.sin(rear_steer)
        return {
            "front_left": vx - yaw_rate * half_track,
            "front_right": vx + yaw_rate * half_track,
            "rear_left": c * (vx - yaw_rate * half_track) - s * yaw_rate * wheelbase,
            "rear_right": c * (vx + yaw_rate * half_track) - s * yaw_rate * wheelbase,
        }

    def wheel_speeds_from_body_twist(
        self,
        vx: float,
        vy: float,
        yaw_rate: float,
        steers: dict[str, float],
    ) -> dict[str, float]:
        speeds = {}
        for name, (rx, ry) in self.positions.items():
            steer = steers[name]
            wheel_vx = vx - yaw_rate * ry
            wheel_vy = vy + yaw_rate * rx
            speeds[name] = math.cos(steer) * wheel_vx + math.sin(steer) * wheel_vy
        return speeds
