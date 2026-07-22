"""Deterministic synthetic human-distance source for Gazebo SSM tests only."""

from __future__ import annotations

import math


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32

    class HumanDistanceSource(Node):
        def __init__(self) -> None:
            super().__init__("canonical_human_distance_source")
            self.declare_parameter("enabled", False)
            self.declare_parameter("topic", "/canonical/human_distance")
            self.declare_parameter("clear_distance", 4.0)
            self.declare_parameter("minimum_distance", 0.35)
            self.declare_parameter("period_sec", 12.0)
            self.enabled = bool(self.get_parameter("enabled").value)
            self.started = self.get_clock().now()
            self.publisher = self.create_publisher(Float32, str(self.get_parameter("topic").value), 10)
            self.create_timer(0.1, self._tick)

        def _tick(self) -> None:
            if not self.enabled:
                return
            elapsed = (self.get_clock().now() - self.started).nanoseconds / 1.0e9
            minimum = max(0.0, float(self.get_parameter("minimum_distance").value))
            clear = max(minimum, float(self.get_parameter("clear_distance").value))
            period = max(0.1, float(self.get_parameter("period_sec").value))
            # A deterministic crossing proxy: clear -> close -> clear.
            distance = minimum + (clear - minimum) * (0.5 + 0.5 * math.cos(2.0 * math.pi * elapsed / period))
            self.publisher.publish(Float32(data=float(distance)))

    rclpy.init()
    node = HumanDistanceSource()
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
