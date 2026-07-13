#!/usr/bin/env python3

import math
from typing import Dict

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class RearSteerDriveController(Node):
    def __init__(self) -> None:
        super().__init__("rear_steer_drive_controller")

        self.declare_parameter("cmd_topic", "/realistic_nav/cmd_vel_request")
        self.declare_parameter(
            "traction_command_topic", "/traction_velocity_controller/commands"
        )
        self.declare_parameter(
            "steering_command_topic", "/rear_steering_position_controller/commands"
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("wheel_radius", 0.18)
        self.declare_parameter("wheelbase", 1.50)
        self.declare_parameter("rear_track", 0.78)
        self.declare_parameter("min_turning_radius", 4.00)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.16)
        self.declare_parameter("allow_rotation_shim", False)
        self.declare_parameter("rotation_shim_linear_speed", 0.025)
        self.declare_parameter("rotation_shim_direction", -1.0)
        self.declare_parameter("max_steering_angle", 0.36)
        self.declare_parameter("max_wheel_speed", 1.20)
        self.declare_parameter("max_steering_rate", 0.14)
        self.declare_parameter("max_wheel_accel", 0.25)
        self.declare_parameter("steering_settle_error", 0.07)
        self.declare_parameter("min_traction_scale_during_steer", 0.45)
        self.declare_parameter("spin_max_steering_angle", 0.24)
        self.declare_parameter("spin_wheel_speed_scale", 0.40)
        self.declare_parameter("coast_speed_threshold", 0.02)
        self.declare_parameter("coast_steering_threshold", 0.05)
        self.declare_parameter("traction_speed_gain", 2.00)
        self.declare_parameter("cmd_timeout_sec", 0.30)
        self.declare_parameter("joint_state_timeout_sec", 0.20)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter(
            "traction_joints",
            ["rear_left_wheel_spin_joint", "rear_right_wheel_spin_joint"],
        )
        self.declare_parameter(
            "steering_joints",
            ["rear_left_steering_joint", "rear_right_steering_joint"],
        )

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.rear_track = float(self.get_parameter("rear_track").value)
        self.min_turning_radius = float(
            self.get_parameter("min_turning_radius").value
        )
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.allow_rotation_shim = bool(
            self.get_parameter("allow_rotation_shim").value
        )
        self.rotation_shim_linear_speed = float(
            self.get_parameter("rotation_shim_linear_speed").value
        )
        self.rotation_shim_direction = float(
            self.get_parameter("rotation_shim_direction").value
        )
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.max_steering_rate = float(
            self.get_parameter("max_steering_rate").value
        )
        self.max_wheel_accel = float(self.get_parameter("max_wheel_accel").value)
        self.steering_settle_error = float(
            self.get_parameter("steering_settle_error").value
        )
        self.min_traction_scale_during_steer = float(
            self.get_parameter("min_traction_scale_during_steer").value
        )
        self.spin_max_steering_angle = float(
            self.get_parameter("spin_max_steering_angle").value
        )
        self.spin_wheel_speed_scale = float(
            self.get_parameter("spin_wheel_speed_scale").value
        )
        self.coast_speed_threshold = float(
            self.get_parameter("coast_speed_threshold").value
        )
        self.coast_steering_threshold = float(
            self.get_parameter("coast_steering_threshold").value
        )
        self.traction_speed_gain = float(
            self.get_parameter("traction_speed_gain").value
        )
        self.max_valid_joint_speed = max(5.0, self.max_wheel_speed * 4.0)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.joint_state_timeout_sec = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.traction_joints = list(self.get_parameter("traction_joints").value)
        self.steering_joints = list(self.get_parameter("steering_joints").value)

        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("traction_command_topic").value),
            10,
        )
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("steering_command_topic").value),
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 20
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 20)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 50)
        self.command_timer = self.create_timer(0.05, self.publish_commands)
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)

        self.last_cmd: Twist | None = None
        self.last_cmd_time = self.get_clock().now()
        self.joint_positions: Dict[str, float] = {}
        self.joint_velocities: Dict[str, float] = {}
        self.joint_position_stamp: Dict[str, float] = {}
        self.joint_velocity_stamp: Dict[str, float] = {}
        self.last_odom_time = self.get_clock().now()
        self.last_command_publish_time = self.get_clock().now()
        self.last_traction_command = [0.0, 0.0]
        self.last_steering_command = [0.0, 0.0]

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info(
            "Rear-steer drive controller active: "
            f"cmd={self.cmd_topic}, traction={self.traction_joints}, steering={self.steering_joints}"
        )

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def joint_state_cb(self, msg: JointState) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                position = msg.position[idx]
                if math.isfinite(position) and abs(position) < 1.5:
                    self.joint_positions[name] = position
                    self.joint_position_stamp[name] = now_sec
                else:
                    self.joint_positions.pop(name, None)
                    self.joint_position_stamp.pop(name, None)
            if idx < len(msg.velocity):
                velocity = msg.velocity[idx]
                if math.isfinite(velocity) and abs(velocity) < self.max_valid_joint_speed:
                    self.joint_velocities[name] = velocity
                    self.joint_velocity_stamp[name] = now_sec
                else:
                    self.joint_velocities.pop(name, None)
                    self.joint_velocity_stamp.pop(name, None)

    def _get_active_cmd(self) -> Twist:
        cmd = Twist()
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if self.last_cmd is None or age > self.cmd_timeout_sec:
            return cmd
        return self.last_cmd

    def _compute_targets(self) -> tuple[list[float], list[float]]:
        cmd = self._get_active_cmd()
        v = clamp(cmd.linear.x, -self.max_linear_speed, self.max_linear_speed)
        w = clamp(cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)
        rotation_shim_active = abs(v) < 1e-4 and abs(w) > 1e-4

        if rotation_shim_active and not self.allow_rotation_shim:
            return [0.0, 0.0], [0.0, 0.0]

        if rotation_shim_active:
            v = self.rotation_shim_direction * self.rotation_shim_linear_speed

        if (
            abs(v) > 1e-4
            and abs(w) > 1e-4
            and self.min_turning_radius > 1e-4
        ):
            max_yaw_rate = abs(v) / self.min_turning_radius
            if abs(w) > max_yaw_rate:
                w = math.copysign(max_yaw_rate, w)

        if abs(v) < 1e-4 or abs(w) < 1e-4:
            steer_left = 0.0
            steer_right = 0.0
        else:
            radius = v / w
            left_denom = max(0.10, abs(radius) - self.rear_track * 0.5)
            right_denom = max(0.10, abs(radius) + self.rear_track * 0.5)
            inner = math.atan(self.wheelbase / left_denom)
            outer = math.atan(self.wheelbase / right_denom)
            # Rear-steer yaw follows w = -v * tan(delta) / L, so the steering sign
            # must depend on both turn direction and travel direction. In reverse,
            # the same desired yaw rate requires the opposite steering sign.
            steering_sign = 1.0 if (-w / v) > 0.0 else -1.0
            if steering_sign > 0.0:
                steer_left = inner
                steer_right = outer
            else:
                steer_left = -outer
                steer_right = -inner

        steer_left = clamp(steer_left, -self.max_steering_angle, self.max_steering_angle)
        steer_right = clamp(steer_right, -self.max_steering_angle, self.max_steering_angle)
        if rotation_shim_active:
            steer_left = clamp(
                steer_left,
                -self.spin_max_steering_angle,
                self.spin_max_steering_angle,
            )
            steer_right = clamp(
                steer_right,
                -self.spin_max_steering_angle,
                self.spin_max_steering_angle,
            )

        steering_severity = max(abs(steer_left), abs(steer_right)) / max(
            self.max_steering_angle, 1e-3
        )
        v *= max(0.55, 1.0 - 0.30 * steering_severity)

        # Gazebo tire scrub and forklift mass make the measured body speed lower than
        # the nominal kinematic target, so apply a calibrated wheel-speed gain here.
        wheel_speed = clamp(
            self.traction_speed_gain * v / self.wheel_radius,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )
        if rotation_shim_active:
            wheel_speed *= self.spin_wheel_speed_scale

        return [wheel_speed, wheel_speed], [steer_left, steer_right]

    def publish_commands(self) -> None:
        traction_target, steering_target = self._compute_targets()
        now = self.get_clock().now()
        dt = (now - self.last_command_publish_time).nanoseconds * 1e-9
        self.last_command_publish_time = now
        # The controller timer runs at 20 Hz; clamp dt so the first non-zero command
        # after a long idle period is still rate-limited instead of jumping instantly.
        dt = min(max(dt, 0.0), 0.05)

        max_wheel_delta = self.max_wheel_accel * dt
        max_steer_delta = self.max_steering_rate * dt

        traction = [
            self._rate_limit(
                current=self.last_traction_command[idx],
                target=traction_target[idx],
                max_delta=max_wheel_delta,
            )
            for idx in range(len(traction_target))
        ]
        steering = [
            self._rate_limit(
                current=self.last_steering_command[idx],
                target=steering_target[idx],
                max_delta=max_steer_delta,
            )
            for idx in range(len(steering_target))
        ]

        current_wheel_speeds = [
            self.joint_velocities.get(joint_name, 0.0) for joint_name in self.traction_joints
        ]
        current_steering_positions = [
            self.joint_positions.get(joint_name, 0.0) for joint_name in self.steering_joints
        ]

        # Rear-steer vehicles can become numerically unstable in Gazebo if full
        # traction is applied while the steer joints are still slewing under load.
        # Bleed traction down during large steering transients instead of cutting it
        # to zero, otherwise Nav2 stalls whenever the steer actuators lag behind.
        steer_errors = []
        for idx, joint_name in enumerate(self.steering_joints):
            current_pos = self.joint_positions.get(joint_name)
            if current_pos is None or not math.isfinite(current_pos):
                continue
            steer_errors.append(abs(steering[idx] - current_pos))
        if steer_errors:
            max_steer_error = max(steer_errors)
            if max_steer_error > self.steering_settle_error:
                vehicle_is_still_rolling = any(
                    math.isfinite(speed) and abs(speed) > self.coast_speed_threshold
                    for speed in current_wheel_speeds
                )
                hard_slew_error = max(self.steering_settle_error * 3.0, 0.12)
                if vehicle_is_still_rolling and max_steer_error > hard_slew_error:
                    traction = [value * 0.25 for value in traction]
                else:
                    error_ratio = clamp(
                        max_steer_error / max(self.max_steering_angle, 1e-3),
                        0.0,
                        1.0,
                    )
                    traction_scale = max(
                        self.min_traction_scale_during_steer,
                        1.0 - error_ratio,
                    )
                    traction = [value * traction_scale for value in traction]

        no_active_request = (
            self.last_cmd is None
            or (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            > self.cmd_timeout_sec
        )
        if no_active_request:
            vehicle_is_still_rolling = any(
                math.isfinite(speed) and abs(speed) > self.coast_speed_threshold
                for speed in current_wheel_speeds
            )
            steering_is_off_center = any(
                math.isfinite(position) and abs(position) > self.coast_steering_threshold
                for position in current_steering_positions
            )
            if vehicle_is_still_rolling and steering_is_off_center:
                traction = [0.0, 0.0]
                steering = list(self.last_steering_command)

        self.last_traction_command = traction
        self.last_steering_command = steering

        traction_msg = Float64MultiArray()
        traction_msg.data = traction
        self.traction_pub.publish(traction_msg)

        steering_msg = Float64MultiArray()
        steering_msg.data = steering
        self.steering_pub.publish(steering_msg)

    @staticmethod
    def _rate_limit(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return target
        delta = clamp(target - current, -max_delta, max_delta)
        return current + delta

    def publish_odometry(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_time = now

        try:
            v_left = self.joint_velocities[self.traction_joints[0]]
            v_right = self.joint_velocities[self.traction_joints[1]]
            steer_left = self.joint_positions[self.steering_joints[0]]
            steer_right = self.joint_positions[self.steering_joints[1]]
        except KeyError:
            return

        if not all(
            math.isfinite(value)
            for value in (v_left, v_right, steer_left, steer_right)
        ):
            return

        now_sec = now.nanoseconds * 1e-9
        measurement_ages = (
            now_sec - self.joint_velocity_stamp.get(self.traction_joints[0], -1e9),
            now_sec - self.joint_velocity_stamp.get(self.traction_joints[1], -1e9),
            now_sec - self.joint_position_stamp.get(self.steering_joints[0], -1e9),
            now_sec - self.joint_position_stamp.get(self.steering_joints[1], -1e9),
        )
        if any(age < 0.0 or age > self.joint_state_timeout_sec for age in measurement_ages):
            return

        linear_velocity = self.wheel_radius * 0.5 * (v_left + v_right)
        steering_angle = 0.5 * (steer_left + steer_right)
        angular_velocity = 0.0
        if abs(math.cos(steering_angle)) > 1e-3:
            angular_velocity = -linear_velocity * math.tan(steering_angle) / self.wheelbase

        self.x += linear_velocity * math.cos(self.yaw) * dt
        self.y += linear_velocity * math.sin(self.yaw) * dt
        self.yaw += angular_velocity * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = odom.header
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = RearSteerDriveController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
