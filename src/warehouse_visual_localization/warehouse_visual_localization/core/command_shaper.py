from dataclasses import dataclass

from warehouse_visual_localization.core.math_utils import clamp


@dataclass
class CommandShaperConfig:
    linear_deadband: float = 0.02
    min_turn_linear_speed: float = 0.08
    turn_command_threshold: float = 0.20
    max_angular_speed: float = 0.45
    max_angular_speed_at_low_linear: float = 0.28
    default_linear_sign: float = 1.0


class CommandShaper:
    """Shared command limiter used by ROS nodes and standalone tests."""

    def __init__(self, config: CommandShaperConfig) -> None:
        self.config = config
        self.preferred_linear_sign = -1.0 if config.default_linear_sign < 0.0 else 1.0

    def observe_input(self, linear_x: float) -> None:
        if abs(linear_x) > max(self.config.linear_deadband, 0.0):
            self.preferred_linear_sign = 1.0 if linear_x >= 0.0 else -1.0

    def shape(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        self.observe_input(linear_x)
        shaped_v = linear_x
        shaped_w = clamp(
            angular_z,
            -abs(self.config.max_angular_speed),
            abs(self.config.max_angular_speed),
        )

        if abs(shaped_w) < max(self.config.turn_command_threshold, 0.0):
            return shaped_v, shaped_w

        if abs(shaped_v) < max(self.config.min_turn_linear_speed, 0.0):
            shaped_v = self.preferred_linear_sign * max(
                self.config.min_turn_linear_speed, 0.0
            )
            shaped_w = clamp(
                shaped_w,
                -abs(self.config.max_angular_speed_at_low_linear),
                abs(self.config.max_angular_speed_at_low_linear),
            )

        return shaped_v, shaped_w

