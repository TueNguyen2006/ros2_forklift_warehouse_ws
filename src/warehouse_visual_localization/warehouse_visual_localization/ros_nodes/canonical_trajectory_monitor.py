"""Opt-in Gazebo/Nav2 monitor backed exclusively by canonical core functions."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, evaluate_corridor
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit, signed_curvature
from warehouse_visual_localization.core.human_ssm import HumanSSMConfig, evaluate_human_ssm
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover
from warehouse_visual_localization.core.types import Pose2D


def _config_section(path: str) -> dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return raw.get("canonical_constraints", {})


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, String
    from visualization_msgs.msg import Marker, MarkerArray

    class CanonicalTrajectoryMonitor(Node):
        """Monitor-only bridge. It never publishes a vehicle command."""

        def __init__(self) -> None:
            super().__init__("canonical_trajectory_monitor")
            default = Path(__file__).resolve().parents[2] / "config" / "core" / "canonical_constraints.yaml"
            for name, default_value in (
                ("enabled", False), ("curvature_speed_limit", False), ("corridor_monitor", False),
                ("rollover_monitor", False), ("human_ssm", False), ("config_path", str(default)),
                ("plan_topic", "/plan"), ("odom_topic", "/odom"),
                ("human_distance_topic", "/canonical/human_distance"),
            ):
                self.declare_parameter(name, default_value)
            self.enabled = bool(self.get_parameter("enabled").value)
            self.use_curvature = bool(self.get_parameter("curvature_speed_limit").value)
            self.use_corridor = bool(self.get_parameter("corridor_monitor").value)
            self.use_rollover = bool(self.get_parameter("rollover_monitor").value)
            self.use_human = bool(self.get_parameter("human_ssm").value)
            section = _config_section(str(self.get_parameter("config_path").value))
            curve = section.get("curvature_speed", {})
            roll = section.get("rollover", {})
            human = section.get("human_ssm", {})
            self.curve_cfg = CurvatureSpeedConfig(**{key: curve[key] for key in CurvatureSpeedConfig.__dataclass_fields__ if key in curve})
            self.roll_cfg = RolloverConfig(**{key: roll[key] for key in RolloverConfig.__dataclass_fields__ if key in roll})
            self.human_cfg = HumanSSMConfig(**{key: human[key] for key in HumanSSMConfig.__dataclass_fields__ if key in human})
            self.body_points = section.get("footprint", {}).get("body_points", [])
            self.path: RosPath | None = None
            self.odom: Odometry | None = None
            self.human_distance: float | None = None
            self.last_speed = 0.0
            self.last_stamp_ns: int | None = None
            self.reference_pub = self.create_publisher(Float32, "/canonical/reference_speed_limit", 10)
            self.margin_pub = self.create_publisher(Float32, "/canonical/corridor_margin", 10)
            self.violation_pub = self.create_publisher(Float32, "/canonical/corridor_violation", 10)
            self.lateral_pub = self.create_publisher(Float32, "/canonical/lateral_acceleration", 10)
            self.utilization_pub = self.create_publisher(Float32, "/canonical/roll_utilization", 10)
            self.zmp_pub = self.create_publisher(Float32, "/canonical/zmp_margin", 10)
            self.roll_safe_pub = self.create_publisher(Bool, "/canonical/roll_safe", 10)
            self.status_pub = self.create_publisher(String, "/canonical/status", 10)
            self.markers_pub = self.create_publisher(MarkerArray, "/canonical/corridor_markers", 10)
            self.create_subscription(RosPath, str(self.get_parameter("plan_topic").value), self._path, 10)
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom, 20)
            self.create_subscription(Float32, str(self.get_parameter("human_distance_topic").value), self._human, 10)
            self.create_timer(0.1, self._tick)
            self.get_logger().info("Canonical trajectory monitor enabled=%s; monitor topics only." % self.enabled)

        def _path(self, message: RosPath) -> None:
            self.path = message

        def _odom(self, message: Odometry) -> None:
            self.odom = message

        def _human(self, message: Float32) -> None:
            self.human_distance = float(message.data) if np.isfinite(message.data) else None

        def _nearest_index(self, points: np.ndarray, pose: Pose2D) -> int:
            return int(np.argmin(np.sum((points - np.array([pose.x, pose.y])) ** 2, axis=1)))

        def _tick(self) -> None:
            if not self.enabled or self.path is None or self.odom is None or not self.path.poses:
                return
            points = np.array([(item.pose.position.x, item.pose.position.y) for item in self.path.poses], dtype=float)
            orientation = self.odom.pose.pose.orientation
            yaw = float(np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)))
            pose = Pose2D(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, yaw)
            speed = float(self.odom.twist.twist.linear.x)
            index = self._nearest_index(points, pose)
            curvature = signed_curvature(points)
            current_curvature = float(curvature[index])
            speed_caps: list[float] = []
            status: dict[str, Any] = {"progress_index": index, "curvature": current_curvature, "speed": speed}
            if self.use_curvature:
                speed_result = curvature_speed_limit(curvature, self.curve_cfg)
                speed_caps.append(float(speed_result.speed_limit[index]))
                status["curvature_speed_limit"] = speed_caps[-1]
            if self.use_human and self.human_distance is not None:
                ssm = evaluate_human_ssm(self.human_distance, abs(speed), self.human_cfg)
                speed_caps.append(ssm.allowed_speed)
                status["human_ssm"] = {"distance": self.human_distance, "allowed_speed": ssm.allowed_speed, "state": ssm.safety_state}
            if speed_caps:
                self.reference_pub.publish(Float32(data=float(min(speed_caps))))
            if self.use_corridor and self.body_points:
                # Benchmark corridor: y in [-1, 1]. Production corridors require an explicit source.
                corridor = evaluate_corridor(self.body_points, pose, CorridorBoundary((0.0, 1.0), 1.0), CorridorBoundary((0.0, 1.0), -1.0))
                self.margin_pub.publish(Float32(data=corridor.minimum_margin))
                self.violation_pub.publish(Float32(data=corridor.maximum_violation))
                status["corridor"] = {"margin": corridor.minimum_margin, "violation": corridor.maximum_violation}
            if self.use_rollover:
                ax = 0.0
                roll = evaluate_rollover(speed, current_curvature, ax, [(-0.6, -0.5), (0.6, -0.5), (0.6, 0.5), (-0.6, 0.5)], self.roll_cfg)
                self.lateral_pub.publish(Float32(data=roll.lateral_acceleration))
                self.utilization_pub.publish(Float32(data=roll.utilization_ratio))
                self.zmp_pub.publish(Float32(data=roll.zmp_margin))
                self.roll_safe_pub.publish(Bool(data=roll.safe))
                status["rollover"] = {"utilization": roll.utilization_ratio, "zmp_margin": roll.zmp_margin, "safe": roll.safe}
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=True)))

    rclpy.init()
    node = CanonicalTrajectoryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
