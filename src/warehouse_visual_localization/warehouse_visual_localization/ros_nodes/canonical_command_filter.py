"""Opt-in command filter for canonical speed caps and linear CBF constraints."""

from __future__ import annotations

import math

from warehouse_visual_localization.core.cbf_filter import filter_linear_cbf_command


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float32, String

    class CanonicalCommandFilter(Node):
        def __init__(self) -> None:
            super().__init__("canonical_command_filter")
            for name, value in (("enabled", False), ("cbf_filter", False), ("input_topic", "/visual_nav/cmd_vel_request"), ("output_topic", "/canonical/cmd_vel_filtered"), ("speed_limit_topic", "/canonical/reference_speed_limit"), ("max_accel", 0.55), ("max_decel", 0.65), ("publish_hz", 20.0), ("max_speed", 0.32), ("max_yaw_rate", 0.32)):
                self.declare_parameter(name, value)
            self.enabled = bool(self.get_parameter("enabled").value)
            self.use_cbf = bool(self.get_parameter("cbf_filter").value)
            self.max_accel = max(0.0, float(self.get_parameter("max_accel").value))
            self.max_decel = max(0.0, float(self.get_parameter("max_decel").value))
            self.max_speed = abs(float(self.get_parameter("max_speed").value))
            self.max_yaw = abs(float(self.get_parameter("max_yaw_rate").value))
            self.limit = self.max_speed
            self.raw = Twist()
            self.output = Twist()
            self.publisher = self.create_publisher(Twist, str(self.get_parameter("output_topic").value), 20)
            self.intervention = self.create_publisher(Float32, "/canonical/cbf_intervention", 10)
            self.active = self.create_publisher(String, "/canonical/cbf_active_constraints", 10)
            self.create_subscription(Twist, str(self.get_parameter("input_topic").value), self._command, 20)
            self.create_subscription(Float32, str(self.get_parameter("speed_limit_topic").value), self._limit, 10)
            self.create_timer(1.0 / max(1.0, float(self.get_parameter("publish_hz").value)), self._tick)

        def _command(self, message: Twist) -> None:
            self.raw = message

        def _limit(self, message: Float32) -> None:
            if math.isfinite(message.data):
                self.limit = max(0.0, min(self.max_speed, float(message.data)))

        def _tick(self) -> None:
            if not self.enabled:
                return
            desired_v = float(self.raw.linear.x)
            desired_w = float(self.raw.angular.z)
            if not math.isfinite(desired_v) or not math.isfinite(desired_w):
                self.output = Twist()
                self.publisher.publish(self.output)
                return
            cap = min(self.max_speed, self.limit)
            if self.use_cbf:
                result = filter_linear_cbf_command([desired_v, desired_w], [[1.0, 0.0], [-1.0, 0.0]], [cap, cap], [-self.max_speed, -self.max_yaw], [self.max_speed, self.max_yaw])
                filtered_v, filtered_w = result.safe_command if result.feasible else (0.0, 0.0)
                self.intervention.publish(Float32(data=result.intervention_magnitude))
                self.active.publish(String(data=','.join(map(str, result.active_constraints))))
            else:
                filtered_v, filtered_w = math.copysign(min(abs(desired_v), cap), desired_v), max(-self.max_yaw, min(self.max_yaw, desired_w))
            dt = 1.0 / max(1.0, float(self.get_parameter("publish_hz").value))
            delta = filtered_v - self.output.linear.x
            rate = self.max_accel if delta >= 0.0 else self.max_decel
            limited_v = self.output.linear.x + max(-rate * dt, min(rate * dt, delta))
            self.output = Twist()
            self.output.linear.x = limited_v
            self.output.angular.z = float(filtered_w)
            self.publisher.publish(self.output)

    rclpy.init()
    node = CanonicalCommandFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
