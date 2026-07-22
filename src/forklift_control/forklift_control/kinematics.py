from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ForkliftKinematicConfig:
    wheelbase: float = 1.35
    wheel_radius: float = 0.16
    steering_limit: float = 0.55
    min_speed_for_steering: float = 0.02
    steering_axle: str = "rear"
    drive_axle: str = "front"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def steering_angle_from_twist(
    linear_speed: float,
    yaw_rate: float,
    config: ForkliftKinematicConfig,
) -> float:
    if abs(linear_speed) < max(config.min_speed_for_steering, 1e-6):
        return 0.0
    angle = math.atan(config.wheelbase * yaw_rate / linear_speed)
    if config.steering_axle == "rear":
        angle = -angle
    return clamp(angle, -abs(config.steering_limit), abs(config.steering_limit))


def wheel_angular_velocity(linear_speed: float, config: ForkliftKinematicConfig) -> float:
    return linear_speed / max(abs(config.wheel_radius), 1e-6)


def yaw_rate_from_state(
    linear_speed: float,
    steering_angle: float,
    config: ForkliftKinematicConfig,
) -> float:
    sign = -1.0 if config.steering_axle == "rear" else 1.0
    return sign * linear_speed * math.tan(steering_angle) / max(abs(config.wheelbase), 1e-6)


def integrate_odometry(
    state: tuple[float, float, float],
    wheel_speed_rad_s: float,
    steering_angle: float,
    dt: float,
    config: ForkliftKinematicConfig,
) -> tuple[float, float, float]:
    x, y, yaw = state
    speed = wheel_speed_rad_s * config.wheel_radius
    yaw_rate = yaw_rate_from_state(speed, steering_angle, config)
    mid_yaw = yaw + 0.5 * yaw_rate * dt
    return (
        x + speed * math.cos(mid_yaw) * dt,
        y + speed * math.sin(mid_yaw) * dt,
        yaw + yaw_rate * dt,
    )
