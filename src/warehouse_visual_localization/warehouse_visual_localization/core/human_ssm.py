"""Speed and Separation Monitoring (SSM) math for simulated human distance."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class HumanSSMConfig:
    braking_deceleration: float = 0.65
    reaction_time: float = 0.20
    control_delay: float = 0.10
    minimum_margin: float = 0.50


@dataclass(frozen=True)
class HumanSSMResult:
    stopping_distance: float
    delay_distance: float
    braking_distance: float
    allowed_speed: float
    safety_state: str


def evaluate_human_ssm(
    separation: float,
    current_speed: float,
    config: HumanSSMConfig | None = None,
    human_approach_speed: float = 0.0,
) -> HumanSSMResult:
    """Evaluate the SSM stopping inequality in metres and seconds.

    ``v^2/(2a_m) + v(t_r+t_c) <= z-z_0``. A positive human approach speed
    consumes additional separation during delay. This is not a human detector;
    it assumes a supplied scalar separation and conservative braking capability.
    """
    cfg = config or HumanSSMConfig()
    values = [separation, current_speed, cfg.braking_deceleration, cfg.reaction_time, cfg.control_delay, cfg.minimum_margin, human_approach_speed]
    if not all(math.isfinite(float(item)) for item in values) or cfg.braking_deceleration <= 0.0 or cfg.reaction_time < 0.0 or cfg.control_delay < 0.0 or cfg.minimum_margin < 0.0 or human_approach_speed < 0.0:
        return HumanSSMResult(0.0, 0.0, 0.0, 0.0, "invalid")
    speed = max(0.0, float(current_speed))
    delay = cfg.reaction_time + cfg.control_delay
    delay_distance = speed * delay
    braking_distance = speed * speed / (2.0 * cfg.braking_deceleration)
    stopping_distance = delay_distance + braking_distance
    available = separation - cfg.minimum_margin - human_approach_speed * delay
    if available <= 0.0:
        return HumanSSMResult(stopping_distance, delay_distance, braking_distance, 0.0, "stop")
    discriminant = max(0.0, (cfg.braking_deceleration * delay) ** 2 + 2.0 * cfg.braking_deceleration * available)
    allowed = max(0.0, -cfg.braking_deceleration * delay + math.sqrt(discriminant))
    state = "clear" if speed <= allowed + 1.0e-9 else "speed_limited"
    return HumanSSMResult(stopping_distance, delay_distance, braking_distance, allowed, state)

