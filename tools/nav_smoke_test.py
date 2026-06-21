#!/usr/bin/env python3

import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SmokeHarness(Node):
    def __init__(self, cmd_topic: str) -> None:
        super().__init__("nav_smoke_harness")
        self.plan_messages = 0
        self.max_plan_poses = 0
        self.nonzero_cmd_count = 0
        self.last_cmd = None
        self.last_odom = None
        self.start_odom = None
        self.max_displacement = 0.0
        self.amcl_pose = None

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.amcl_state_client = self.create_client(GetState, "/amcl/get_state")
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.create_subscription(Path, "/plan", self.plan_cb, 20)
        self.create_subscription(Twist, cmd_topic, self.cmd_cb, 20)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_cb, 20
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
        }

    def plan_cb(self, msg: Path) -> None:
        self.plan_messages += 1
        self.max_plan_poses = max(self.max_plan_poses, len(msg.poses))

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = {"linear_x": msg.linear.x, "angular_z": msg.angular.z}
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.nonzero_cmd_count += 1

    def odom_cb(self, msg: Odometry) -> None:
        self.last_odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_rate": msg.twist.twist.angular.z,
            "speed": msg.twist.twist.linear.x,
        }
        if self.start_odom is not None:
            dx = self.last_odom["x"] - self.start_odom["x"]
            dy = self.last_odom["y"] - self.start_odom["y"]
            self.max_displacement = max(self.max_displacement, math.hypot(dx, dy))

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]
        self.initial_pose_pub.publish(msg)


def wait_for(node: Node, predicate, timeout_sec: float, description: str) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(f"Timed out waiting for {description}")


def wait_for_lifecycle_active(
    node: SmokeHarness, client, timeout_sec: float, description: str
) -> None:
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise TimeoutError(f"Timed out waiting for {description} state service")

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        future = client.call_async(GetState.Request())
        wait_for(node, future.done, 5.0, f"{description} state response")
        label = future.result().current_state.label.lower()
        if label == "active":
            return
        time.sleep(0.2)

    raise TimeoutError(f"Timed out waiting for {description} to become active")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial-x", type=float, required=True)
    parser.add_argument("--initial-y", type=float, required=True)
    parser.add_argument("--initial-yaw", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, required=True)
    parser.add_argument("--cmd-topic", type=str, required=True)
    parser.add_argument("--require-amcl", action="store_true")
    parser.add_argument("--require-bt-active", action="store_true")
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = SmokeHarness(args.cmd_topic)
    started = time.monotonic()
    summary = {
        "result": "ERROR",
        "timed_out": False,
        "plan_messages": 0,
        "max_plan_poses": 0,
        "nonzero_cmd_count": 0,
        "max_displacement_m": 0.0,
        "start_odom": None,
        "final_odom": None,
        "last_cmd": None,
        "elapsed_sec": 0.0,
    }

    try:
        if not node.nav_client.wait_for_server(timeout_sec=60.0):
            raise TimeoutError("Timed out waiting for navigate_to_pose action server")

        if args.require_amcl:
            wait_for_lifecycle_active(node, node.amcl_state_client, 60.0, "amcl")

            for _ in range(5):
                node.publish_initial_pose(args.initial_x, args.initial_y, args.initial_yaw)
                rclpy.spin_once(node, timeout_sec=0.2)

            wait_for(node, lambda: node.amcl_pose is not None, 30.0, "amcl pose")
        if args.require_bt_active:
            wait_for_lifecycle_active(node, node.bt_state_client, 60.0, "bt_navigator")
        wait_for(node, lambda: node.last_odom is not None, 20.0, "odom data")
        node.start_odom = node.last_odom.copy() if node.last_odom else None

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation = yaw_to_quaternion(args.goal_yaw)

        send_goal_future = node.nav_client.send_goal_async(goal)
        wait_for(node, send_goal_future.done, 10.0, "goal acceptance")
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("navigate_to_pose goal was not accepted")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout_sec
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                cancel_future = goal_handle.cancel_goal_async()
                wait_for(node, cancel_future.done, 5.0, "goal cancellation")
                summary["timed_out"] = True
                break

        if result_future.done():
            status = result_future.result().status
        elif summary["timed_out"]:
            status = GoalStatus.STATUS_CANCELED
        else:
            status = GoalStatus.STATUS_UNKNOWN

        summary["result"] = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "FAILED",
        }.get(status, f"STATUS_{status}")
    except Exception as exc:  # pragma: no cover - smoke-test reporting path
        summary["error"] = str(exc)
    finally:
        summary["plan_messages"] = node.plan_messages
        summary["max_plan_poses"] = node.max_plan_poses
        summary["nonzero_cmd_count"] = node.nonzero_cmd_count
        summary["max_displacement_m"] = round(node.max_displacement, 3)
        summary["start_odom"] = node.start_odom
        summary["final_odom"] = node.last_odom
        summary["last_cmd"] = node.last_cmd
        summary["elapsed_sec"] = round(time.monotonic() - started, 3)
        print(json.dumps(summary))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
