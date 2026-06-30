#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WaitForMapTfNode(Node):
    def __init__(self):
        super().__init__("wait_for_map_tf")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("require_global_frame", True)
        self.declare_parameter("timeout_sec", 45.0)
        self.declare_parameter("poll_period_sec", 0.25)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.require_global_frame = bool(
            self.get_parameter("require_global_frame").value
        )
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.poll_period_sec = float(self.get_parameter("poll_period_sec").value)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)

    def wait(self) -> bool:
        start_time = time.monotonic()
        warned = False
        if self.require_global_frame:
            self.get_logger().info(
                "Waiting for pose chain %s -> %s -> %s..."
                % (self.global_frame, self.odom_frame, self.robot_frame)
            )
        else:
            self.get_logger().info(
                "Waiting for odometry pose chain %s -> %s..."
                % (self.odom_frame, self.robot_frame)
            )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.poll_period_sec)
            try:
                odom_to_base = self.buffer.lookup_transform(
                    self.odom_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                odom_translation = odom_to_base.transform.translation
                if self.require_global_frame:
                    map_to_odom = self.buffer.lookup_transform(
                        self.global_frame,
                        self.odom_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.1),
                    )
                    map_translation = map_to_odom.transform.translation
                    self.get_logger().info(
                        "Visual pose chain ready: %s -> %s | x=%.3f y=%.3f, %s -> %s | x=%.3f y=%.3f"
                        % (
                            self.global_frame,
                            self.odom_frame,
                            map_translation.x,
                            map_translation.y,
                            self.odom_frame,
                            self.robot_frame,
                            odom_translation.x,
                            odom_translation.y,
                        )
                    )
                else:
                    self.get_logger().info(
                        "Odometry pose chain ready: %s -> %s | x=%.3f y=%.3f"
                        % (
                            self.odom_frame,
                            self.robot_frame,
                            odom_translation.x,
                            odom_translation.y,
                        )
                    )
                return True
            except TransformException:
                if (
                    self.timeout_sec > 0.0
                    and not warned
                    and time.monotonic() - start_time >= self.timeout_sec
                ):
                    self.get_logger().warning(
                        "Pose chain %s -> %s -> %s is still unavailable after %.1fs. "
                        "Nav2 will keep waiting until localization is really ready."
                        % (
                            self.global_frame,
                            self.odom_frame,
                            self.robot_frame,
                            self.timeout_sec,
                        )
                    )
                    warned = True
                continue

        self.get_logger().warning("Shutting down while waiting for visual pose chain.")
        return False


def main():
    rclpy.init()
    node = WaitForMapTfNode()
    try:
        success = node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
