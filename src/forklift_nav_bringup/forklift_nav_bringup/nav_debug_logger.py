import math
import json
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


class NavDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__("nav_debug_logger")

        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("log_dir", str(Path.home() / ".ros" / "forklift_nav_logs"))
        self.declare_parameter("log_prefix", "baseline_nav")
        self.declare_parameter("sample_period_sec", 0.25)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")

        log_dir = Path(self.get_parameter("log_dir").value).expanduser()
        log_prefix = str(self.get_parameter("log_prefix").value)
        sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)

        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = log_dir / f"{log_prefix}_{timestamp}.jsonl"
        self.log_handle = self.log_path.open("a", encoding="utf-8")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_odom: Optional[Odometry] = None
        self.last_cmd: Optional[Twist] = None
        self.last_goal: Optional[PoseStamped] = None
        self.last_plan_summary = None
        self.tf_warning_keys = set()

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_cb, 20)
        self.create_subscription(PoseStamped, self.goal_topic, self._goal_cb, 20)
        self.create_subscription(NavPath, self.plan_topic, self._plan_cb, 20)
        self.timer = self.create_timer(max(sample_period_sec, 0.05), self._sample)

        self._write_event(
            "logger_started",
            {
                "log_path": str(self.log_path),
                "global_frame": self.global_frame,
                "odom_frame": self.odom_frame,
                "robot_frame": self.robot_frame,
            },
        )
        self.get_logger().info(f"Writing nav debug log to {self.log_path}")

    def _ros_time(self) -> dict:
        now = self.get_clock().now().to_msg()
        return {"sec": int(now.sec), "nanosec": int(now.nanosec)}

    def _write_event(self, event_type: str, payload: dict) -> None:
        record = {
            "event": event_type,
            "stamp": self._ros_time(),
            **payload,
        }
        self.log_handle.write(json.dumps(record, ensure_ascii=True) + "\n")
        self.log_handle.flush()

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.last_goal = msg
        self._write_event(
            "goal",
            {
                "frame_id": msg.header.frame_id,
                "pose": {
                    "x": float(msg.pose.position.x),
                    "y": float(msg.pose.position.y),
                    "z": float(msg.pose.position.z),
                    "yaw": quaternion_to_yaw(msg.pose.orientation),
                },
            },
        )

    def _plan_cb(self, msg: NavPath) -> None:
        if not msg.poses:
            summary = {"frame_id": msg.header.frame_id, "poses": 0}
        else:
            first_pose = msg.poses[0].pose
            last_pose = msg.poses[-1].pose
            summary = {
                "frame_id": msg.header.frame_id,
                "poses": len(msg.poses),
                "start": {
                    "x": float(first_pose.position.x),
                    "y": float(first_pose.position.y),
                    "yaw": quaternion_to_yaw(first_pose.orientation),
                },
                "end": {
                    "x": float(last_pose.position.x),
                    "y": float(last_pose.position.y),
                    "yaw": quaternion_to_yaw(last_pose.orientation),
                },
            }
        self.last_plan_summary = summary
        self._write_event("plan", summary)

    def _lookup_pose(self, target_frame: str, source_frame: str) -> Optional[dict]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            warning_key = f"{target_frame}->{source_frame}:{exc}"
            if warning_key not in self.tf_warning_keys:
                self.tf_warning_keys.add(warning_key)
                self._write_event(
                    "tf_warning",
                    {
                        "target_frame": target_frame,
                        "source_frame": source_frame,
                        "message": str(exc),
                    },
                )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
            "yaw": quaternion_to_yaw(rotation),
        }

    def _sample(self) -> None:
        sample = {
            "map_to_robot": self._lookup_pose(self.global_frame, self.robot_frame),
            "odom_to_robot": self._lookup_pose(self.odom_frame, self.robot_frame),
            "last_plan": self.last_plan_summary,
        }

        if self.last_odom is not None:
            pose = self.last_odom.pose.pose
            twist = self.last_odom.twist.twist
            sample["odom"] = {
                "frame_id": self.last_odom.header.frame_id,
                "child_frame_id": self.last_odom.child_frame_id,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw": quaternion_to_yaw(pose.orientation),
                "linear_x": float(twist.linear.x),
                "angular_z": float(twist.angular.z),
            }

        if self.last_cmd is not None:
            sample["cmd_vel"] = {
                "linear_x": float(self.last_cmd.linear.x),
                "angular_z": float(self.last_cmd.angular.z),
            }

        if self.last_goal is not None:
            sample["goal_frame"] = self.last_goal.header.frame_id

        self._write_event("sample", sample)

    def destroy_node(self) -> bool:
        self._write_event("logger_stopped", {})
        self.log_handle.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavDebugLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
