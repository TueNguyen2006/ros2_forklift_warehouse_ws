#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TfDebugNode(Node):
    def __init__(self):
        super().__init__("tf_debug_node")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("period_sec", 2.0)

        self.global_frame = self.get_parameter("global_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        period = float(self.get_parameter("period_sec").value)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(period, self._on_timer)

    def _lookup_status(self, target: str, source: str) -> str:
        try:
            transform = self.buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            translation = transform.transform.translation
            return (
                f"ok x={translation.x:.3f} y={translation.y:.3f} "
                f"z={translation.z:.3f}"
            )
        except TransformException as exc:
            return f"missing ({exc})"

    def _on_timer(self):
        self.get_logger().info(
            "TF status | map->odom: %s | odom->base: %s | map->base: %s"
            % (
                self._lookup_status(self.global_frame, self.odom_frame),
                self._lookup_status(self.odom_frame, self.base_frame),
                self._lookup_status(self.global_frame, self.base_frame),
            )
        )


def main():
    rclpy.init()
    node = TfDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
