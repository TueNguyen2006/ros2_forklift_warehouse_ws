#!/usr/bin/env python3

import json
import math
import time
from pathlib import Path
from typing import List

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(float(yaw))
    return pose


def status_to_text(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(status, f"STATUS_{status}")


class VisualRouteRunner(Node):
    def __init__(self) -> None:
        super().__init__("visual_route_runner")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("result_file", "")
        self.declare_parameter("goal_timeout_sec", 240.0)
        self.declare_parameter("server_wait_sec", 120.0)
        self.declare_parameter("tf_wait_sec", 120.0)
        self.declare_parameter("goal_accept_timeout_sec", 10.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        self.scenario_file = str(self.get_parameter("scenario_file").value)
        self.result_file = str(self.get_parameter("result_file").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.server_wait_sec = float(self.get_parameter("server_wait_sec").value)
        self.tf_wait_sec = float(self.get_parameter("tf_wait_sec").value)
        self.goal_accept_timeout_sec = float(
            self.get_parameter("goal_accept_timeout_sec").value
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)

        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.navigate_through_poses_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_servers(self) -> None:
        self.get_logger().info("Waiting for Nav2 action servers...")
        if not self.navigate_to_pose_client.wait_for_server(
            timeout_sec=self.server_wait_sec
        ):
            raise RuntimeError("Timed out waiting for /navigate_to_pose action server.")
        if not self.navigate_through_poses_client.wait_for_server(
            timeout_sec=self.server_wait_sec
        ):
            raise RuntimeError(
                "Timed out waiting for /navigate_through_poses action server."
            )
        self.get_logger().info("Nav2 action servers are ready.")

    def wait_for_pose_chain(self) -> None:
        self.get_logger().info(
            f"Waiting for TF {self.global_frame} -> {self.robot_frame}..."
        )
        deadline = time.monotonic() + self.tf_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.25)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                translation = transform.transform.translation
                self.get_logger().info(
                    "TF ready: %s -> %s | x=%.3f y=%.3f z=%.3f"
                    % (
                        self.global_frame,
                        self.robot_frame,
                        translation.x,
                        translation.y,
                        translation.z,
                    )
                )
                return
            except TransformException:
                continue

        raise RuntimeError(
            f"Timed out waiting for TF {self.global_frame} -> {self.robot_frame}."
        )

    def _send_navigate_to_pose(self, goal: dict) -> dict:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose(goal["x"], goal["y"], goal["yaw"])
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        feedback_state = {
            "distance_remaining": None,
            "navigation_time_sec": None,
            "number_of_recoveries": None,
        }

        def feedback_callback(feedback_msg) -> None:
            feedback = feedback_msg.feedback
            feedback_state["distance_remaining"] = float(
                getattr(feedback, "distance_remaining", 0.0)
            )
            navigation_time = getattr(feedback, "navigation_time", None)
            if navigation_time is not None:
                feedback_state["navigation_time_sec"] = (
                    float(navigation_time.sec)
                    + float(navigation_time.nanosec) / 1e9
                )
            feedback_state["number_of_recoveries"] = int(
                getattr(feedback, "number_of_recoveries", 0)
            )

        goal_handle = None
        accept_deadline = time.monotonic() + self.goal_accept_timeout_sec
        while rclpy.ok() and time.monotonic() < accept_deadline:
            goal_future = self.navigate_to_pose_client.send_goal_async(
                goal_msg, feedback_callback=feedback_callback
            )
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            self.get_logger().info(
                "NavigateToPose goal not accepted yet, retrying while Nav2 finishes activation..."
            )
            time.sleep(0.5)
        if goal_handle is None or not goal_handle.accepted:
            return {
                "accepted": False,
                "status": "REJECTED",
                "elapsed_sec": 0.0,
                "distance_remaining": None,
                "number_of_recoveries": None,
            }

        start_time = time.monotonic()
        result_future = goal_handle.get_result_async()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            if time.monotonic() - start_time > self.goal_timeout_sec:
                self.get_logger().warning(
                    f"Goal {goal.get('name', 'unnamed')} timed out, canceling."
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                break

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        result = result_future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

        return {
            "accepted": True,
            "status": status_to_text(status),
            "elapsed_sec": round(time.monotonic() - start_time, 3),
            "distance_remaining": feedback_state["distance_remaining"],
            "navigation_time_sec": feedback_state["navigation_time_sec"],
            "number_of_recoveries": feedback_state["number_of_recoveries"],
        }

    def _send_navigate_through_poses(self, segment: dict) -> dict:
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [
            make_pose(pose["x"], pose["y"], pose["yaw"])
            for pose in segment.get("poses", [])
        ]
        now = self.get_clock().now().to_msg()
        for pose in goal_msg.poses:
            pose.header.stamp = now

        feedback_state = {
            "distance_remaining": None,
            "navigation_time_sec": None,
            "number_of_recoveries": None,
            "number_of_poses_remaining": None,
        }

        def feedback_callback(feedback_msg) -> None:
            feedback = feedback_msg.feedback
            feedback_state["distance_remaining"] = float(
                getattr(feedback, "distance_remaining", 0.0)
            )
            navigation_time = getattr(feedback, "navigation_time", None)
            if navigation_time is not None:
                feedback_state["navigation_time_sec"] = (
                    float(navigation_time.sec)
                    + float(navigation_time.nanosec) / 1e9
                )
            feedback_state["number_of_recoveries"] = int(
                getattr(feedback, "number_of_recoveries", 0)
            )
            feedback_state["number_of_poses_remaining"] = int(
                getattr(feedback, "number_of_poses_remaining", 0)
            )

        goal_handle = None
        accept_deadline = time.monotonic() + self.goal_accept_timeout_sec
        while rclpy.ok() and time.monotonic() < accept_deadline:
            goal_future = self.navigate_through_poses_client.send_goal_async(
                goal_msg, feedback_callback=feedback_callback
            )
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            self.get_logger().info(
                "NavigateThroughPoses goal not accepted yet, retrying while Nav2 finishes activation..."
            )
            time.sleep(0.5)
        if goal_handle is None or not goal_handle.accepted:
            return {
                "accepted": False,
                "status": "REJECTED",
                "elapsed_sec": 0.0,
                "distance_remaining": None,
                "number_of_recoveries": None,
                "number_of_poses_remaining": None,
            }

        start_time = time.monotonic()
        result_future = goal_handle.get_result_async()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            if time.monotonic() - start_time > self.goal_timeout_sec:
                self.get_logger().warning(
                    f"Segment {segment.get('name', 'unnamed')} timed out, canceling."
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                break

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        result = result_future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

        return {
            "accepted": True,
            "status": status_to_text(status),
            "elapsed_sec": round(time.monotonic() - start_time, 3),
            "distance_remaining": feedback_state["distance_remaining"],
            "navigation_time_sec": feedback_state["navigation_time_sec"],
            "number_of_recoveries": feedback_state["number_of_recoveries"],
            "number_of_poses_remaining": feedback_state["number_of_poses_remaining"],
        }

    def run(self) -> Path:
        if not self.scenario_file:
            raise RuntimeError("scenario_file parameter is required.")

        with Path(self.scenario_file).open("r", encoding="utf-8") as handle:
            scenario = yaml.safe_load(handle) or {}

        self.wait_for_servers()
        self.wait_for_pose_chain()

        summary = {
            "scenario_file": self.scenario_file,
            "started_at": time.time(),
            "segments": [],
        }

        for segment in scenario.get("segments", []):
            segment_name = segment.get("name", "unnamed_segment")
            segment_type = segment.get("type", "navigate_through_poses")
            self.get_logger().info(
                f"Running segment {segment_name} with type {segment_type}."
            )

            if segment_type == "navigate_to_pose":
                result = self._send_navigate_to_pose(segment)
            elif segment_type == "navigate_through_poses":
                result = self._send_navigate_through_poses(segment)
            else:
                raise RuntimeError(f"Unsupported segment type: {segment_type}")

            segment_summary = {
                "name": segment_name,
                "type": segment_type,
                "result": result,
            }
            if "poses" in segment:
                segment_summary["poses"] = segment["poses"]
            else:
                segment_summary["pose"] = {
                    "x": segment["x"],
                    "y": segment["y"],
                    "yaw": segment["yaw"],
                }

            summary["segments"].append(segment_summary)

            if result["status"] != "SUCCEEDED":
                self.get_logger().warning(
                    f"Segment {segment_name} finished with status {result['status']}."
                )
                break

        summary["finished_at"] = time.time()
        summary["success_count"] = sum(
            1
            for segment in summary["segments"]
            if segment["result"]["status"] == "SUCCEEDED"
        )
        summary["failure_count"] = len(summary["segments"]) - summary["success_count"]

        output_path = (
            Path(self.result_file)
            if self.result_file
            else Path(self.scenario_file).with_suffix(".results.json")
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.get_logger().info(f"Scenario result written to {output_path}")
        return output_path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualRouteRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
