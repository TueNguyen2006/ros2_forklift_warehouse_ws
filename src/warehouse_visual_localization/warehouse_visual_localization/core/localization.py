import math
from dataclasses import dataclass

from warehouse_visual_localization.core.math_utils import wrap_angle
from warehouse_visual_localization.core.types import Pose2D


@dataclass
class EKF2DConfig:
    process_xy: float = 0.025
    process_yaw: float = 0.015
    vision_xy: float = 0.080
    vision_yaw: float = 0.060
    wheel_xy: float = 0.030
    wheel_yaw: float = 0.030


class EKF2D:
    """Tiny diagonal EKF for standalone localization tests.

    It mirrors the idea used in robot_localization: wheel odometry predicts
    smooth motion and visual odometry corrects drift.
    """

    def __init__(self, pose: Pose2D | None = None, config: EKF2DConfig | None = None) -> None:
        self.pose = pose or Pose2D()
        self.config = config or EKF2DConfig()
        self.var_x = 0.04
        self.var_y = 0.04
        self.var_yaw = 0.04

    def predict(self, v: float, yaw_rate: float, dt: float) -> Pose2D:
        self.pose.x += v * math.cos(self.pose.yaw) * dt
        self.pose.y += v * math.sin(self.pose.yaw) * dt
        self.pose.yaw = wrap_angle(self.pose.yaw + yaw_rate * dt)
        self.var_x += self.config.process_xy * dt
        self.var_y += self.config.process_xy * dt
        self.var_yaw += self.config.process_yaw * dt
        return self.pose

    def update_visual_pose(self, measurement: Pose2D) -> Pose2D:
        self.pose.x, self.var_x = self._update_scalar(self.pose.x, self.var_x, measurement.x, self.config.vision_xy)
        self.pose.y, self.var_y = self._update_scalar(self.pose.y, self.var_y, measurement.y, self.config.vision_xy)
        yaw_error = wrap_angle(measurement.yaw - self.pose.yaw)
        corrected, self.var_yaw = self._update_scalar(0.0, self.var_yaw, yaw_error, self.config.vision_yaw)
        self.pose.yaw = wrap_angle(self.pose.yaw + corrected)
        return self.pose

    @staticmethod
    def _update_scalar(value: float, variance: float, measurement: float, measurement_variance: float) -> tuple[float, float]:
        gain = variance / max(variance + measurement_variance, 1e-9)
        value = value + gain * (measurement - value)
        variance = (1.0 - gain) * variance
        return value, variance

