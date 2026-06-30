#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSourceMonitor(Node):
    def __init__(self):
        super().__init__("pose_source_monitor")
        self.declare_parameter("pose_source", "rgbd_odom")
        self.declare_parameter("consumer_name", "visual_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("require_map_frame", False)
        self.declare_parameter("period_sec", 2.0)

        self.pose_source = str(self.get_parameter("pose_source").value)
        self.consumer_name = str(self.get_parameter("consumer_name").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.require_map_frame = bool(self.get_parameter("require_map_frame").value)
        period = float(self.get_parameter("period_sec").value)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(period, self._on_timer)

    def _has_transform(self, target: str, source: str) -> bool:
        try:
            self.buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
            return True
        except TransformException:
            return False

    def _on_timer(self):
        has_odom = self._has_transform(self.odom_frame, self.robot_frame)
        has_map = self._has_transform(self.global_frame, self.odom_frame)
        state = (
            "ready"
            if has_odom and (has_map or not self.require_map_frame)
            else "waiting_for_tf"
        )
        self.get_logger().info(
            "Pose source=%s | consumer=%s | state=%s | map->odom=%s | odom->base=%s"
            % (
                self.pose_source,
                self.consumer_name,
                state,
                "ok" if has_map else "missing",
                "ok" if has_odom else "missing",
            )
        )


def main():
    rclpy.init()
    node = PoseSourceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
