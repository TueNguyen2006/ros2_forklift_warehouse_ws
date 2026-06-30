#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node


class TwistWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("twist_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("hold_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("use_stamped_output", False)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.hold_timeout = Duration(
            seconds=float(self.get_parameter("hold_timeout_sec").value)
        )
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.use_stamped_output = bool(self.get_parameter("use_stamped_output").value)

        if self.use_stamped_output:
            self.publisher = self.create_publisher(TwistStamped, self.output_topic, 20)
        else:
            self.publisher = self.create_publisher(Twist, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._input_cb, 20
        )
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.last_msg = Twist()
        self.last_stamp = None

        self.get_logger().info(
            "Twist watchdog bridging %s -> %s as %s (timeout=%.2fs)"
            % (
                self.input_topic,
                self.output_topic,
                "TwistStamped" if self.use_stamped_output else "Twist",
                self.hold_timeout.nanoseconds / 1e9,
            )
        )

    def _input_cb(self, msg: Twist) -> None:
        self.last_msg = msg
        self.last_stamp = self.get_clock().now()

    def _publish_twist(self, msg: Twist) -> None:
        if self.use_stamped_output:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.twist = msg
            self.publisher.publish(stamped)
            return
        self.publisher.publish(msg)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_stamp is None or (now - self.last_stamp) > self.hold_timeout:
            self._publish_twist(Twist())
            return
        self._publish_twist(self.last_msg)


def main() -> None:
    rclpy.init()
    node = TwistWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_twist(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
