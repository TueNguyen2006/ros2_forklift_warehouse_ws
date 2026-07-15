from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class VehicleState:
    pose: Pose2D
    v: float = 0.0
    yaw_rate: float = 0.0
    steer: float = 0.0
    ax: float = 0.0
    slip_ratio: float = 0.0


@dataclass
class ControlCommand:
    v: float = 0.0
    yaw_rate: float = 0.0
    steer: float = 0.0
    accel: float = 0.0


@dataclass
class VehicleParams:
    wheelbase: float = 1.25
    track_width: float = 0.82
    mass: float = 1400.0
    yaw_inertia: float = 900.0
    cg_to_front_axle: float = 0.55
    cg_height: float = 0.45
    max_steer: float = 0.62
    max_steer_rate: float = 1.2
    max_speed: float = 0.35
    max_reverse_speed: float = 0.18
    max_accel: float = 0.55
    max_decel: float = 0.65
    gravity: float = 9.81
    drive_axle: str = "rear"
    steer_axle: str = "rear"


@dataclass
class VehicleForces:
    longitudinal: float = 0.0
    lateral: float = 0.0
    normal_front: float = 0.0
    normal_rear: float = 0.0
    yaw_moment: float = 0.0

