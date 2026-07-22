from dataclasses import dataclass

from forklift_control.kinematics import clamp


@dataclass(frozen=True)
class ActuatorLimitConfig:
    max_speed: float = 1.0
    max_reverse_speed: float = 0.5
    max_accel: float = 0.45
    max_decel: float = 0.65
    max_steering_angle: float = 0.55
    max_steering_rate: float = 0.65
    command_timeout_sec: float = 0.25
    steering_dead_zone: float = 0.0
    control_delay_sec: float = 0.0
    backlash: float = 0.0


@dataclass(frozen=True)
class ActuatorCommand:
    speed: float = 0.0
    steering_angle: float = 0.0


def limit_command(
    previous: ActuatorCommand,
    target: ActuatorCommand,
    dt: float,
    config: ActuatorLimitConfig,
) -> ActuatorCommand:
    dt = max(dt, 0.0)
    speed_target = clamp(
        target.speed,
        -abs(config.max_reverse_speed),
        abs(config.max_speed),
    )
    speed_delta = speed_target - previous.speed
    speed_rate = config.max_accel if speed_delta >= 0.0 else config.max_decel
    speed = previous.speed + clamp(speed_delta, -abs(speed_rate) * dt, abs(speed_rate) * dt)

    steering_target = clamp(
        target.steering_angle,
        -abs(config.max_steering_angle),
        abs(config.max_steering_angle),
    )
    if abs(steering_target) < abs(config.steering_dead_zone):
        steering_target = 0.0
    steering_delta = steering_target - previous.steering_angle
    steering = previous.steering_angle + clamp(
        steering_delta,
        -abs(config.max_steering_rate) * dt,
        abs(config.max_steering_rate) * dt,
    )
    return ActuatorCommand(speed=speed, steering_angle=steering)
