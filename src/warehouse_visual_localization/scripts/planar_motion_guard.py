#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class PlanarMotionGuard(Node):
    def __init__(self) -> None:
        super().__init__("planar_motion_guard")
        self.declare_parameter("input_topic", "/visual_nav/cmd_vel_request")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("hold_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("linear_deadband", 0.02)
        self.declare_parameter("min_turn_linear_speed", 0.08)
        self.declare_parameter("turn_command_threshold", 0.20)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("max_angular_speed_at_low_linear", 0.28)
        self.declare_parameter("default_linear_sign", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.hold_timeout = Duration(
            seconds=float(self.get_parameter("hold_timeout_sec").value)
        )
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.linear_deadband = max(
            float(self.get_parameter("linear_deadband").value), 0.0
        )
        self.min_turn_linear_speed = max(
            float(self.get_parameter("min_turn_linear_speed").value), 0.0
        )
        self.turn_command_threshold = max(
            float(self.get_parameter("turn_command_threshold").value), 0.0
        )
        self.max_angular_speed = max(
            float(self.get_parameter("max_angular_speed").value), 0.0
        )
        self.max_angular_speed_at_low_linear = max(
            float(self.get_parameter("max_angular_speed_at_low_linear").value), 0.0
        )
        default_linear_sign = float(self.get_parameter("default_linear_sign").value)
        self.preferred_linear_sign = -1.0 if default_linear_sign < 0.0 else 1.0

        self.publisher = self.create_publisher(Twist, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._input_cb, 20
        )
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.last_msg = Twist()
        self.last_stamp = None

        self.get_logger().info(
            "Planar motion guard bridging %s -> %s | min_turn_linear=%.2f wz_max=%.2f wz_low_linear=%.2f"
            % (
                self.input_topic,
                self.output_topic,
                self.min_turn_linear_speed,
                self.max_angular_speed,
                self.max_angular_speed_at_low_linear,
            )
        )

    def _input_cb(self, msg: Twist) -> None:
        self.last_msg = msg
        self.last_stamp = self.get_clock().now()
        if math.fabs(msg.linear.x) > self.linear_deadband:
            self.preferred_linear_sign = 1.0 if msg.linear.x >= 0.0 else -1.0

    def _shape_twist(self, raw: Twist) -> Twist:
        shaped = Twist()
        shaped.linear.x = raw.linear.x
        shaped.angular.z = clamp(
            raw.angular.z, -self.max_angular_speed, self.max_angular_speed
        )

        if math.fabs(shaped.angular.z) < self.turn_command_threshold:
            return shaped

        if math.fabs(shaped.linear.x) < self.min_turn_linear_speed:
            shaped.linear.x = self.preferred_linear_sign * self.min_turn_linear_speed
            shaped.angular.z = clamp(
                shaped.angular.z,
                -self.max_angular_speed_at_low_linear,
                self.max_angular_speed_at_low_linear,
            )

        return shaped

    def _publish(self, msg: Twist) -> None:
        self.publisher.publish(msg)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_stamp is None or (now - self.last_stamp) > self.hold_timeout:
            self._publish(Twist())
            return
        self._publish(self._shape_twist(self.last_msg))


def main() -> None:
    rclpy.init()
    node = PlanarMotionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
