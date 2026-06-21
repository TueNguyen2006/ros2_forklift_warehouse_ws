#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class DrivetrainHarness(Node):
    def __init__(self) -> None:
        super().__init__("drivetrain_smoke_test")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.last_odom = None
        self.last_joint_state = {}
        self.samples = []
        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 50)

    def odom_cb(self, msg: Odometry) -> None:
        yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.last_odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw,
            "speed": msg.twist.twist.linear.x,
            "yaw_rate": msg.twist.twist.angular.z,
        }

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def joint_state_cb(self, msg: JointState) -> None:
        state = {}
        for index, name in enumerate(msg.name):
            velocity = msg.velocity[index] if index < len(msg.velocity) else None
            position = msg.position[index] if index < len(msg.position) else None
            state[name] = {"velocity": velocity, "position": position}
        self.last_joint_state = state

    def capture(self, label: str) -> None:
        snapshot = {"label": label, "odom": self.last_odom}
        self.samples.append(snapshot)


def angle_diff(a: float, b: float) -> float:
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def drive_stage(
    node: DrivetrainHarness,
    label: str,
    linear_x: float,
    angular_z: float,
    duration_sec: float,
) -> dict:
    node.capture(f"{label}_start")
    started = time.monotonic()
    wheel_velocity_samples = []
    while time.monotonic() - started < duration_sec:
        node.publish_cmd(linear_x, angular_z)
        rclpy.spin_once(node, timeout_sec=0.05)
        left = node.last_joint_state.get("left_wheel_joint", {}).get("velocity")
        right = node.last_joint_state.get("right_wheel_joint", {}).get("velocity")
        if left is not None and right is not None:
            wheel_velocity_samples.append((left, right))
    node.publish_cmd(0.0, 0.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.capture(f"{label}_end")

    start = node.samples[-2]["odom"]
    end = node.samples[-1]["odom"]
    if start is None or end is None:
        return {"label": label, "error": "missing odom"}

    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    distance = math.hypot(dx, dy)
    heading_change = angle_diff(end["yaw"], start["yaw"])
    direction_projection = (
        dx * math.cos(start["yaw"]) + dy * math.sin(start["yaw"])
    )
    lateral_projection = (
        -dx * math.sin(start["yaw"]) + dy * math.cos(start["yaw"])
    )
    avg_left_velocity = None
    avg_right_velocity = None
    if wheel_velocity_samples:
        avg_left_velocity = sum(sample[0] for sample in wheel_velocity_samples) / len(
            wheel_velocity_samples
        )
        avg_right_velocity = sum(sample[1] for sample in wheel_velocity_samples) / len(
            wheel_velocity_samples
        )
    return {
        "label": label,
        "command": {"linear_x": linear_x, "angular_z": angular_z},
        "distance_m": round(distance, 3),
        "forward_projection_m": round(direction_projection, 3),
        "lateral_projection_m": round(lateral_projection, 3),
        "heading_change_rad": round(heading_change, 3),
        "avg_left_wheel_velocity": (
            None if avg_left_velocity is None else round(avg_left_velocity, 3)
        ),
        "avg_right_wheel_velocity": (
            None if avg_right_velocity is None else round(avg_right_velocity, 3)
        ),
        "start": start,
        "end": end,
    }


def wait_for_odom(node: DrivetrainHarness, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_odom is not None:
            return
    raise TimeoutError("Timed out waiting for /odom")


def main() -> None:
    rclpy.init()
    node = DrivetrainHarness()
    summary = {"result": "ERROR", "stages": []}
    try:
        wait_for_odom(node, 30.0)
        for _ in range(20):
            node.publish_cmd(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        summary["stages"].append(drive_stage(node, "forward", 0.4, 0.0, 4.0))
        summary["stages"].append(drive_stage(node, "rotate_left", 0.0, 0.5, 4.0))
        summary["stages"].append(drive_stage(node, "arc_left", 0.25, 0.25, 4.0))
        summary["result"] = "OK"
    except Exception as exc:  # pragma: no cover - smoke reporting path
        summary["error"] = str(exc)
    finally:
        node.publish_cmd(0.0, 0.0)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        print(json.dumps(summary))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
