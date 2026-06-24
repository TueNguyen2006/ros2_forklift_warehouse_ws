import math
from pathlib import Path
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
import yaml


DEFAULT_PROFILE_CONFIG = {
    "global": {
        "gravity": 9.81,
        "support_half_width": 0.52,
        "lateral_safety_factor": 0.42,
        "max_accel": 0.60,
        "max_yaw_rate": 0.70,
        "max_yaw_accel": 0.90,
        "lift_cg_height_gain": 0.85,
        "full_penalty_height": 1.20,
        "min_height_velocity_factor": 0.25,
        "min_speed_for_curvature": 0.05,
        "overheight_creep_speed": 0.15,
        "overheight_yaw_rate": 0.18,
        "emergency_roll_deg": 8.0,
        "emergency_pitch_deg": 10.0,
    },
    "profiles": {
        "EMPTY": {
            "mass": 1400.0,
            "cg_height": 0.45,
            "cg_offset_x": -0.05,
            "max_travel_lift_height": 0.25,
            "max_linear_speed": 0.85,
            "max_decel": 0.80,
            "max_curvature": 0.55,
        }
    },
}


def clamp_magnitude(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class StabilityGuard(Node):
    def __init__(self) -> None:
        super().__init__("stability_guard")

        self.declare_parameter("input_topic", "cmd_vel_smoothed")
        self.declare_parameter("output_topic", "cmd_vel_stability")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("profile_config_path", "")
        self.declare_parameter("load_profile", "EMPTY")
        self.declare_parameter("lift_joint_name", "fork_base_joint")

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.profile_config_path = (
            self.get_parameter("profile_config_path").get_parameter_value().string_value
        )
        self.load_profile = self.get_parameter("load_profile").get_parameter_value().string_value
        self.lift_joint_name = (
            self.get_parameter("lift_joint_name").get_parameter_value().string_value
        )

        self.global_cfg, self.profiles = self._load_profile_config(self.profile_config_path)
        self.log_timestamps: Dict[str, float] = {}
        self.active_profile = self.profiles.get(self.load_profile, self.profiles["EMPTY"])

        self.current_speed = 0.0
        self.current_yaw_rate = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.lift_height = 0.0
        self.lift_joint_index = None
        self.last_output = Twist()
        self.last_cmd_time = None

        self.cmd_sub = self.create_subscription(Twist, self.input_topic, self.cmd_callback, 20)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 20)
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 20
        )
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 20)

        if self.load_profile not in self.profiles:
            self.get_logger().warning(
                f"Profile '{self.load_profile}' not found, falling back to EMPTY."
            )
            self.load_profile = "EMPTY"
            self.active_profile = self.profiles["EMPTY"]

    def _load_profile_config(self, path: str) -> Tuple[Dict, Dict]:
        if not path:
            return (
                DEFAULT_PROFILE_CONFIG["global"],
                DEFAULT_PROFILE_CONFIG["profiles"],
            )

        cfg_path = Path(path)
        if not cfg_path.exists():
            self.get_logger().warning(
                f"Profile config '{cfg_path}' not found, using built-in defaults."
            )
            return (
                DEFAULT_PROFILE_CONFIG["global"],
                DEFAULT_PROFILE_CONFIG["profiles"],
            )

        with cfg_path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}

        global_cfg = DEFAULT_PROFILE_CONFIG["global"].copy()
        global_cfg.update(loaded.get("global", {}))

        profiles = DEFAULT_PROFILE_CONFIG["profiles"].copy()
        profiles.update(loaded.get("profiles", {}))
        return global_cfg, profiles

    def odom_callback(self, msg: Odometry) -> None:
        self.current_speed = msg.twist.twist.linear.x
        self.current_yaw_rate = msg.twist.twist.angular.z

    def imu_callback(self, msg: Imu) -> None:
        roll, pitch, _ = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.current_roll = roll
        self.current_pitch = pitch

    def joint_state_callback(self, msg: JointState) -> None:
        index = self.lift_joint_index
        if index is None:
            try:
                index = msg.name.index(self.lift_joint_name)
            except ValueError:
                return
            self.lift_joint_index = index
        if index < len(msg.position):
            self.lift_height = max(0.0, msg.position[index])

    def _throttled_warn(self, key: str, message: str, throttle_sec: float = 2.0) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        last = self.log_timestamps.get(key, 0.0)
        if now - last >= throttle_sec:
            self.get_logger().warning(message)
            self.log_timestamps[key] = now

    def _get_profile(self) -> Dict:
        return self.active_profile

    def _height_velocity_factor(self, profile: Dict) -> float:
        penalty_height = max(
            float(self.global_cfg.get("full_penalty_height", 1.0)),
            float(profile.get("max_travel_lift_height", 0.25)),
        )
        min_factor = float(self.global_cfg.get("min_height_velocity_factor", 0.25))
        height_ratio = min(self.lift_height / max(penalty_height, 1e-3), 1.0)
        return max(min_factor, 1.0 - (1.0 - min_factor) * height_ratio)

    def _apply_rate_limit(
        self, current: float, target: float, dt: float, max_accel: float, max_decel: float
    ) -> float:
        if dt <= 0.0:
            return target

        delta = target - current
        limit = max_accel if delta >= 0.0 else max_decel
        return current + clamp_magnitude(delta, abs(limit) * dt)

    def cmd_callback(self, msg: Twist) -> None:
        profile = self._get_profile()
        output = Twist()
        output.linear.x = msg.linear.x
        output.angular.z = msg.angular.z

        emergency_roll = math.radians(float(self.global_cfg.get("emergency_roll_deg", 8.0)))
        emergency_pitch = math.radians(float(self.global_cfg.get("emergency_pitch_deg", 10.0)))
        if abs(self.current_roll) > emergency_roll or abs(self.current_pitch) > emergency_pitch:
            self._throttled_warn(
                "emergency_stop",
                "Emergency stop: roll/pitch exceeded configured stability envelope.",
            )
            self.cmd_pub.publish(Twist())
            self.last_output = Twist()
            self.last_cmd_time = self.get_clock().now()
            return

        height_factor = self._height_velocity_factor(profile)
        max_speed = float(profile.get("max_linear_speed", 0.8)) * height_factor
        max_yaw_rate = float(self.global_cfg.get("max_yaw_rate", 0.7)) * height_factor
        max_decel = float(profile.get("max_decel", 0.8)) * height_factor
        max_accel = float(self.global_cfg.get("max_accel", 0.6)) * height_factor
        max_yaw_accel = float(self.global_cfg.get("max_yaw_accel", 0.9)) * height_factor
        min_speed_for_curvature = float(self.global_cfg.get("min_speed_for_curvature", 0.05))

        output.linear.x = clamp_magnitude(output.linear.x, max_speed)
        output.angular.z = clamp_magnitude(output.angular.z, max_yaw_rate)

        curvature = abs(output.angular.z) / max(abs(output.linear.x), min_speed_for_curvature)
        max_curvature = float(profile.get("max_curvature", 0.55))
        if curvature > max_curvature and abs(output.linear.x) > 1e-3:
            output.angular.z = math.copysign(max_curvature * abs(output.linear.x), output.angular.z)
            self._throttled_warn(
                "curvature_limit",
                "Curvature limited by configured forklift stability envelope.",
            )

        dynamic_cg_height = float(profile.get("cg_height", 0.45)) * (
            1.0
            + float(self.global_cfg.get("lift_cg_height_gain", 0.85))
            * (1.0 - height_factor)
        )
        support_half_width = float(self.global_cfg.get("support_half_width", 0.52))
        lateral_safety_factor = float(self.global_cfg.get("lateral_safety_factor", 0.42))
        gravity = float(self.global_cfg.get("gravity", 9.81))

        curvature = abs(output.angular.z) / max(abs(output.linear.x), min_speed_for_curvature)
        if curvature > 1e-3:
            max_lateral_accel = (
                gravity * support_half_width / max(dynamic_cg_height, 1e-3)
            ) * lateral_safety_factor
            max_speed_from_curvature = math.sqrt(max(max_lateral_accel / curvature, 0.0))
            if abs(output.linear.x) > max_speed_from_curvature:
                output.linear.x = math.copysign(
                    max_speed_from_curvature,
                    output.linear.x,
                )
                self._throttled_warn(
                    "lateral_limit",
                    "Velocity reduced to keep lateral acceleration inside the anti-tip envelope.",
                )

        if self.lift_height > float(profile.get("max_travel_lift_height", 0.25)):
            creep_speed = float(self.global_cfg.get("overheight_creep_speed", 0.15))
            overheight_yaw_rate = float(self.global_cfg.get("overheight_yaw_rate", 0.18))
            output.linear.x = clamp_magnitude(output.linear.x, creep_speed)
            output.angular.z = clamp_magnitude(output.angular.z, overheight_yaw_rate)

        now = self.get_clock().now()
        dt = 0.05
        if self.last_cmd_time is not None:
            dt = max((now - self.last_cmd_time).nanoseconds / 1e9, 1e-3)

        output.linear.x = self._apply_rate_limit(
            self.last_output.linear.x,
            output.linear.x,
            dt,
            max_accel,
            max_decel,
        )
        output.angular.z = self._apply_rate_limit(
            self.last_output.angular.z,
            output.angular.z,
            dt,
            max_yaw_accel,
            max_yaw_accel,
        )

        self.cmd_pub.publish(output)
        self.last_output = output
        self.last_cmd_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StabilityGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
