#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class StartupMotionProbe(Node):
    def __init__(self):
        super().__init__("startup_motion_probe")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("initial_delay_sec", 2.0)
        self.declare_parameter("forward_duration_sec", 2.0)
        self.declare_parameter("reverse_duration_sec", 0.0)
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("angular_speed", 0.18)
        self.declare_parameter("publish_hz", 10.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.initial_delay_sec = float(self.get_parameter("initial_delay_sec").value)
        self.forward_duration_sec = float(
            self.get_parameter("forward_duration_sec").value
        )
        self.reverse_duration_sec = float(
            self.get_parameter("reverse_duration_sec").value
        )
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_period_sec = 1.0 / max(
            float(self.get_parameter("publish_hz").value), 1.0
        )
        self.done = False

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.start_time = None
        self.timer = self.create_timer(self.publish_period_sec, self._on_timer)

        self.get_logger().info(
            "Startup motion probe armed on %s (delay=%.1fs, forward=%.1fs, reverse=%.1fs)"
            % (
                self.cmd_vel_topic,
                self.initial_delay_sec,
                self.forward_duration_sec,
                self.reverse_duration_sec,
            )
        )

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9
        forward_end = self.initial_delay_sec + self.forward_duration_sec
        reverse_end = forward_end + self.reverse_duration_sec

        if elapsed < self.initial_delay_sec:
            self._publish(0.0, 0.0)
            return

        if elapsed < forward_end:
            self._publish(self.linear_speed, self.angular_speed)
            return

        if elapsed < reverse_end:
            self._publish(-self.linear_speed, -self.angular_speed)
            return

        self._publish(0.0, 0.0)
        self.get_logger().info("Startup motion probe complete.")
        self.timer.cancel()
        self.done = True


def main():
    rclpy.init()
    node = StartupMotionProbe()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.done:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
