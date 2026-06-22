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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def point_distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


class NavDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__("nav_debug_logger")

        self.declare_parameter("log_dir", str(Path.home() / ".ros" / "forklift_nav_logs"))
        self.declare_parameter("log_prefix", "baseline_nav")
        self.declare_parameter("sample_period_sec", 0.25)
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("console_status", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("raw_cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")

        log_dir = Path(self.get_parameter("log_dir").value).expanduser()
        log_prefix = str(self.get_parameter("log_prefix").value)
        sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self.status_period_sec = max(float(self.get_parameter("status_period_sec").value), 0.2)
        self.console_status = bool(self.get_parameter("console_status").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.raw_cmd_vel_topic = str(self.get_parameter("raw_cmd_vel_topic").value)
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
        self.last_raw_cmd: Optional[Twist] = None
        self.last_goal: Optional[PoseStamped] = None
        self.last_plan_summary = None
        self.last_plan_points = []
        self.last_plan_cumulative = []
        self.last_plan_stamp_sec: Optional[float] = None
        self.plan_update_count = 0
        self.last_status_time_sec = 0.0
        self.tf_warning_keys = set()

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_cb, 20)
        self.create_subscription(Twist, self.raw_cmd_vel_topic, self._raw_cmd_cb, 20)
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
                "odom_topic": self.odom_topic,
                "cmd_vel_topic": self.cmd_vel_topic,
                "raw_cmd_vel_topic": self.raw_cmd_vel_topic,
                "plan_topic": self.plan_topic,
            },
        )
        self.get_logger().info(f"Writing nav debug log to {self.log_path}")

    def _ros_time(self) -> dict:
        now = self.get_clock().now().to_msg()
        return {"sec": int(now.sec), "nanosec": int(now.nanosec)}

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

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

    def _raw_cmd_cb(self, msg: Twist) -> None:
        self.last_raw_cmd = msg

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
        self.plan_update_count += 1
        self.last_plan_stamp_sec = self._now_sec()
        self.last_plan_points = []
        self.last_plan_cumulative = []

        if not msg.poses:
            summary = {"frame_id": msg.header.frame_id, "poses": 0}
        else:
            first_pose = msg.poses[0].pose
            last_pose = msg.poses[-1].pose
            total_length = 0.0
            last_xy = None
            for pose_stamped in msg.poses:
                pose = pose_stamped.pose
                x = float(pose.position.x)
                y = float(pose.position.y)
                yaw = quaternion_to_yaw(pose.orientation)
                self.last_plan_points.append({"x": x, "y": y, "yaw": yaw})
                if last_xy is None:
                    self.last_plan_cumulative.append(0.0)
                else:
                    total_length += point_distance(last_xy[0], last_xy[1], x, y)
                    self.last_plan_cumulative.append(total_length)
                last_xy = (x, y)
            summary = {
                "frame_id": msg.header.frame_id,
                "poses": len(msg.poses),
                "plan_update_count": self.plan_update_count,
                "total_length": total_length,
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

    def _compute_tracking(self, robot_pose: Optional[dict]) -> Optional[dict]:
        if robot_pose is None or not self.last_plan_points:
            return None

        if self.last_plan_summary and self.last_plan_summary.get("frame_id") != self.global_frame:
            return {
                "available": False,
                "reason": "plan_frame_mismatch",
                "plan_frame": self.last_plan_summary.get("frame_id"),
            }

        rx = float(robot_pose["x"])
        ry = float(robot_pose["y"])
        robot_yaw = float(robot_pose["yaw"])

        nearest_index = min(
            range(len(self.last_plan_points)),
            key=lambda idx: point_distance(
                rx,
                ry,
                self.last_plan_points[idx]["x"],
                self.last_plan_points[idx]["y"],
            ),
        )
        nearest = self.last_plan_points[nearest_index]
        cross_track_error = point_distance(rx, ry, nearest["x"], nearest["y"])

        if nearest_index + 1 < len(self.last_plan_points):
            next_point = self.last_plan_points[nearest_index + 1]
            segment_yaw = math.atan2(
                next_point["y"] - nearest["y"],
                next_point["x"] - nearest["x"],
            )
        else:
            segment_yaw = nearest["yaw"]

        heading_error = normalize_angle(segment_yaw - robot_yaw)
        plan_total_length = self.last_plan_cumulative[-1] if self.last_plan_cumulative else 0.0
        distance_along_path = (
            self.last_plan_cumulative[nearest_index] if self.last_plan_cumulative else 0.0
        )
        remaining_path_length = max(plan_total_length - distance_along_path, 0.0)
        progress_ratio = (
            distance_along_path / plan_total_length if plan_total_length > 1e-6 else 1.0
        )

        tracking = {
            "available": True,
            "nearest_index": int(nearest_index),
            "plan_points": len(self.last_plan_points),
            "cross_track_error": cross_track_error,
            "heading_error": heading_error,
            "distance_along_path": distance_along_path,
            "remaining_path_length": remaining_path_length,
            "progress_ratio": progress_ratio,
        }

        if self.last_goal is not None:
            tracking["goal_distance"] = point_distance(
                rx,
                ry,
                float(self.last_goal.pose.position.x),
                float(self.last_goal.pose.position.y),
            )

        if self.last_plan_stamp_sec is not None:
            tracking["plan_age_sec"] = max(self._now_sec() - self.last_plan_stamp_sec, 0.0)

        return tracking

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
        now_sec = self._now_sec()
        map_to_robot = self._lookup_pose(self.global_frame, self.robot_frame)
        odom_to_robot = self._lookup_pose(self.odom_frame, self.robot_frame)
        sample = {
            "map_to_robot": map_to_robot,
            "odom_to_robot": odom_to_robot,
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

        if self.last_raw_cmd is not None:
            sample["cmd_vel_raw"] = {
                "linear_x": float(self.last_raw_cmd.linear.x),
                "angular_z": float(self.last_raw_cmd.angular.z),
            }

        if self.last_cmd is not None:
            sample["cmd_vel"] = {
                "linear_x": float(self.last_cmd.linear.x),
                "angular_z": float(self.last_cmd.angular.z),
            }

        if self.last_cmd is not None and self.last_raw_cmd is not None:
            sample["cmd_tracking"] = {
                "linear_delta": float(self.last_cmd.linear.x - self.last_raw_cmd.linear.x),
                "angular_delta": float(self.last_cmd.angular.z - self.last_raw_cmd.angular.z),
            }

        if self.last_goal is not None:
            sample["goal_frame"] = self.last_goal.header.frame_id

        tracking = self._compute_tracking(map_to_robot)
        if tracking is not None:
            sample["tracking"] = tracking

        self._write_event("sample", sample)

        if self.console_status and now_sec - self.last_status_time_sec >= self.status_period_sec:
            tracking_summary = sample.get("tracking", {})
            cmd_summary = sample.get("cmd_vel", {})
            odom_summary = sample.get("odom", {})
            status_parts = [
                f"plans={self.plan_update_count}",
            ]
            if tracking_summary.get("available"):
                status_parts.extend(
                    [
                        f"cte={tracking_summary['cross_track_error']:.2f}m",
                        f"remain={tracking_summary['remaining_path_length']:.2f}m",
                        f"head_err={tracking_summary['heading_error']:.2f}rad",
                    ]
                )
                if "goal_distance" in tracking_summary:
                    status_parts.append(f"goal={tracking_summary['goal_distance']:.2f}m")
            if cmd_summary:
                status_parts.append(
                    f"cmd=({cmd_summary['linear_x']:.2f},{cmd_summary['angular_z']:.2f})"
                )
            if odom_summary:
                status_parts.append(
                    f"odom=({odom_summary['linear_x']:.2f},{odom_summary['angular_z']:.2f})"
                )
            self.get_logger().info(" | ".join(status_parts))
            self.last_status_time_sec = now_sec

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
