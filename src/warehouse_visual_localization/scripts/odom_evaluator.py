#!/usr/bin/env python3
import csv
import math
import os
from dataclasses import dataclass
from typing import Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compose_pose_2d(base: "Pose2DStamped", local: "Pose2DStamped") -> "Pose2DStamped":
    cos_yaw = math.cos(base.yaw)
    sin_yaw = math.sin(base.yaw)
    x = base.x + (cos_yaw * local.x) - (sin_yaw * local.y)
    y = base.y + (sin_yaw * local.x) + (cos_yaw * local.y)
    return Pose2DStamped(
        stamp_sec=local.stamp_sec,
        x=x,
        y=y,
        yaw=normalize_angle(base.yaw + local.yaw),
    )


def inverse_pose_2d(pose: "Pose2DStamped") -> "Pose2DStamped":
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    x = -((cos_yaw * pose.x) + (sin_yaw * pose.y))
    y = -((-sin_yaw * pose.x) + (cos_yaw * pose.y))
    return Pose2DStamped(
        stamp_sec=pose.stamp_sec,
        x=x,
        y=y,
        yaw=normalize_angle(-pose.yaw),
    )


@dataclass
class Pose2DStamped:
    stamp_sec: float
    x: float
    y: float
    yaw: float


class OdomEvaluator(Node):
    def __init__(self):
        super().__init__("odom_evaluator")
        self.declare_parameter("estimated_odom_topic", "/odom")
        self.declare_parameter("ground_truth_topic", "/gazebo/model_states")
        self.declare_parameter("robot_model_name", "forklift_baseline")
        self.declare_parameter(
            "csv_path",
            os.path.join(
                os.path.expanduser("~"),
                "ros2_forklift_warehouse_artifacts",
                "results",
                "rtabmap_rgbd_eval.csv",
            ),
        )
        self.declare_parameter("lost_gap_sec", 1.5)

        self.estimated_topic = self.get_parameter("estimated_odom_topic").value
        self.ground_truth_topic = self.get_parameter("ground_truth_topic").value
        self.robot_model_name = self.get_parameter("robot_model_name").value
        self.csv_path = self.get_parameter("csv_path").value
        self.lost_gap_sec = float(self.get_parameter("lost_gap_sec").value)

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "timestamp",
                "gt_x",
                "gt_y",
                "gt_yaw",
                "est_x",
                "est_y",
                "est_yaw",
                "position_error",
                "yaw_error",
                "rpe",
                "fps",
                "processing_latency",
            ]
        )
        self.csv_file.flush()

        self.latest_gt: Optional[Pose2DStamped] = None
        self.prev_est: Optional[Pose2DStamped] = None
        self.prev_gt: Optional[Pose2DStamped] = None
        self.alignment_world_from_est: Optional[Pose2DStamped] = None
        self.last_est_wall_time: Optional[float] = None
        self.currently_lost = False

        self.position_errors = []
        self.yaw_errors = []
        self.rpes = []
        self.fps_values = []
        self.latencies = []
        self.tracking_lost_count = 0

        self.create_subscription(ModelStates, self.ground_truth_topic, self._on_gt, 10)
        self.create_subscription(Odometry, self.estimated_topic, self._on_estimated_odom, 50)
        self.create_timer(0.5, self._monitor_tracking)

    def _clock_now(self) -> float:
        now = self.get_clock().now().nanoseconds
        return now / 1e9

    def _on_gt(self, msg: ModelStates):
        if self.robot_model_name not in msg.name:
            return
        index = msg.name.index(self.robot_model_name)
        pose = msg.pose[index]
        self.latest_gt = Pose2DStamped(
            stamp_sec=self._clock_now(),
            x=pose.position.x,
            y=pose.position.y,
            yaw=quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _on_estimated_odom(self, msg: Odometry):
        if self.latest_gt is None:
            return

        stamp = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        est = Pose2DStamped(
            stamp_sec=stamp,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
        )
        gt = self.latest_gt

        if self.alignment_world_from_est is None:
            self.alignment_world_from_est = compose_pose_2d(gt, inverse_pose_2d(est))

        aligned_est = compose_pose_2d(self.alignment_world_from_est, est)

        position_error = math.hypot(aligned_est.x - gt.x, aligned_est.y - gt.y)
        yaw_error = abs(normalize_angle(aligned_est.yaw - gt.yaw))
        now_sec = self._clock_now()
        latency = max(0.0, now_sec - stamp)
        fps = 0.0
        rpe = 0.0

        if self.prev_est is not None and self.prev_gt is not None:
            est_dx = aligned_est.x - self.prev_est.x
            est_dy = aligned_est.y - self.prev_est.y
            gt_dx = gt.x - self.prev_gt.x
            gt_dy = gt.y - self.prev_gt.y
            rpe = math.hypot(est_dx - gt_dx, est_dy - gt_dy)

            dt = est.stamp_sec - self.prev_est.stamp_sec
            if dt > 1e-6:
                fps = 1.0 / dt
                self.fps_values.append(fps)
                if dt > self.lost_gap_sec and not self.currently_lost:
                    self.tracking_lost_count += 1
                    self.currently_lost = True
            else:
                fps = 0.0

        self.position_errors.append(position_error)
        self.yaw_errors.append(yaw_error)
        self.rpes.append(rpe)
        self.latencies.append(latency)
        self.last_est_wall_time = now_sec
        self.currently_lost = False

        self.writer.writerow(
            [
                f"{stamp:.6f}",
                f"{gt.x:.6f}",
                f"{gt.y:.6f}",
                f"{gt.yaw:.6f}",
                f"{aligned_est.x:.6f}",
                f"{aligned_est.y:.6f}",
                f"{aligned_est.yaw:.6f}",
                f"{position_error:.6f}",
                f"{yaw_error:.6f}",
                f"{rpe:.6f}",
                f"{fps:.3f}",
                f"{latency:.6f}",
            ]
        )
        self.csv_file.flush()

        self.prev_est = aligned_est
        self.prev_gt = gt

    def _monitor_tracking(self):
        if self.last_est_wall_time is None or self.currently_lost:
            return
        if (self._clock_now() - self.last_est_wall_time) > self.lost_gap_sec:
            self.tracking_lost_count += 1
            self.currently_lost = True
            self.get_logger().warning("Tracking gap detected.")

    def destroy_node(self):
        self._log_summary()
        if not self.csv_file.closed:
            self.csv_file.close()
        return super().destroy_node()

    def _mean(self, values):
        return sum(values) / len(values) if values else 0.0

    def _log_summary(self):
        self.get_logger().info(
            "Evaluation summary | mean_ATE=%.4f m | mean_RPE=%.4f m | "
            "mean_yaw_error=%.4f rad | mean_FPS=%.2f | mean_latency=%.4f s | "
            "tracking_lost_count=%d | csv=%s"
            % (
                self._mean(self.position_errors),
                self._mean(self.rpes),
                self._mean(self.yaw_errors),
                self._mean(self.fps_values),
                self._mean(self.latencies),
                self.tracking_lost_count,
                self.csv_path,
            )
        )


def main():
    rclpy.init()
    node = OdomEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
