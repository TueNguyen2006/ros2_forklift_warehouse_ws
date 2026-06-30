#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_goal_bridge")
        self.declare_parameter("goal_topic", "/gazebo/nav_goal_pose")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("use_current_yaw", False)
        self.declare_parameter("publish_goal_pose_topic", True)
        self.declare_parameter("goal_pose_topic", "/goal_pose")

        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        nav_action_name = str(self.get_parameter("nav_action_name").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, nav_action_name)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.on_goal_clicked,
            10,
        )

        self.goal_pose_publisher = None
        if bool(self.get_parameter("publish_goal_pose_topic").value):
            self.goal_pose_publisher = self.create_publisher(
                PoseStamped,
                str(self.get_parameter("goal_pose_topic").value),
                10,
            )

        self.get_logger().info(
            f"Listening for Gazebo goals on {goal_topic} and sending to {nav_action_name}"
        )

    def lookup_current_yaw(self) -> float:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal_frame,
                self.robot_frame,
                rclpy.time.Time(),
            )
            return quaternion_to_yaw(transform.transform.rotation)
        except TransformException as exc:
            self.get_logger().warn(
                f"Failed to lookup {self.goal_frame}->{self.robot_frame}, using yaw=0: {exc}"
            )
            return 0.0

    def on_goal_clicked(self, msg: PoseStamped) -> None:
        yaw = self.lookup_current_yaw() if bool(self.get_parameter("use_current_yaw").value) else 0.0
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.goal_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(msg.pose.position.x)
        goal_msg.pose.position.y = float(msg.pose.position.y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z, goal_msg.pose.orientation.w = yaw_to_quaternion(yaw)

        if self.goal_pose_publisher is not None:
            self.goal_pose_publisher.publish(goal_msg)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose action server is not available yet")
            return

        self.get_logger().info(
            "Sending Gazebo-clicked goal "
            f"x={goal_msg.pose.position.x:.2f} y={goal_msg.pose.position.y:.2f} yaw={yaw:.2f}"
        )
        future = self.action_client.send_goal_async(nav_goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().warn("No goal handle returned by NavigateToPose")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("Gazebo-clicked goal was rejected by Nav2")
            return

        self.get_logger().info("Gazebo-clicked goal accepted by Nav2")


def main() -> None:
    rclpy.init()
    node = GazeboGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
