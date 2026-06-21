import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("publish_topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_delay_sec", 2.0)
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("publish_count", 10)

        publish_topic = self.get_parameter("publish_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.initial_x = float(self.get_parameter("initial_x").value)
        self.initial_y = float(self.get_parameter("initial_y").value)
        self.initial_yaw = float(self.get_parameter("initial_yaw").value)
        self.initial_delay_sec = float(self.get_parameter("initial_delay_sec").value)
        self.publish_count = int(self.get_parameter("publish_count").value)

        publish_period_sec = max(float(self.get_parameter("publish_period_sec").value), 0.1)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, publish_topic, 10)
        self.started_at_sec = self.get_clock().now().nanoseconds / 1e9
        self.publish_attempts = 0
        self.timer = self.create_timer(publish_period_sec, self._publish_initial_pose)

    def _publish_initial_pose(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.started_at_sec < self.initial_delay_sec:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]

        self.publisher.publish(msg)
        self.publish_attempts += 1

        if self.publish_attempts == 1:
            self.get_logger().info(
                f"Publishing initial pose at ({self.initial_x:.2f}, {self.initial_y:.2f}, yaw={self.initial_yaw:.2f})."
            )

        if self.publish_attempts >= self.publish_count:
            self.get_logger().info("Initial pose publication complete.")
            self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
