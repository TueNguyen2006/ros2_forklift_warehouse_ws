"""ROS diagnostic adapter for the canonical constraints core APIs."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from warehouse_visual_localization.core.canonical_constraints import CanonicalConstraintConfig, CanonicalConstraintEvaluator
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig
from warehouse_visual_localization.core.human_ssm import HumanSSMConfig
from warehouse_visual_localization.core.rollover_safety import RolloverConfig
from warehouse_visual_localization.core.types import Pose2D


def _section(values: dict[str, Any], name: str) -> dict[str, Any]:
    item = values.get(name, {})
    return item if isinstance(item, dict) else {}


def load_evaluator(path: str) -> CanonicalConstraintEvaluator:
    """Load only the dedicated canonical YAML; never changes existing configs."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    root = _section(raw, "canonical_constraints")
    gates = CanonicalConstraintConfig(
        enabled=bool(root.get("enabled", False)),
        curvature_speed_enabled=bool(_section(root, "curvature_speed").get("enabled", True)),
        footprint_enabled=bool(_section(root, "footprint").get("enabled", True)),
        rollover_enabled=bool(_section(root, "rollover").get("enabled", True)),
        human_ssm_enabled=bool(_section(root, "human_ssm").get("enabled", False)),
        docking_enabled=bool(_section(root, "docking").get("enabled", False)),
        visual_servo_enabled=bool(_section(root, "visual_servo").get("enabled", False)),
        cbf_enabled=bool(_section(root, "cbf").get("enabled", False)),
    )
    curvature = _section(root, "curvature_speed")
    rollover = _section(root, "rollover")
    human = _section(root, "human_ssm")
    return CanonicalConstraintEvaluator(
        gates,
        CurvatureSpeedConfig(**{key: curvature[key] for key in CurvatureSpeedConfig.__dataclass_fields__ if key in curvature}),
        RolloverConfig(**{key: rollover[key] for key in RolloverConfig.__dataclass_fields__ if key in rollover}),
        HumanSSMConfig(**{key: human[key] for key in HumanSSMConfig.__dataclass_fields__ if key in human}),
    )


def main() -> None:
    """Run an opt-in monitor that has no command output path."""
    import rclpy
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.node import Node
    from std_msgs.msg import Float32, String

    class CanonicalSafetyMonitor(Node):
        def __init__(self) -> None:
            super().__init__("canonical_safety_monitor")
            default_config = Path(__file__).resolve().parents[2] / "config" / "core" / "canonical_constraints.yaml"
            self.declare_parameter("enabled", False)
            self.declare_parameter("config_path", str(default_config))
            self.declare_parameter("plan_topic", "/plan")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("human_distance_topic", "/forklift/canonical_constraints/human_distance")
            self.declare_parameter("status_topic", "/forklift/canonical_constraints/status")
            self.declare_parameter("reference_speed_topic", "/forklift/reference_speed_limit")
            self.enabled = bool(self.get_parameter("enabled").value)
            self.evaluator = load_evaluator(str(self.get_parameter("config_path").value))
            self.latest_path: RosPath | None = None
            self.latest_odom: Odometry | None = None
            self.human_distance: float | None = None
            self.status_publisher = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
            self.speed_publisher = self.create_publisher(Float32, str(self.get_parameter("reference_speed_topic").value), 10)
            self.create_subscription(RosPath, str(self.get_parameter("plan_topic").value), self._path_callback, 10)
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_callback, 20)
            self.create_subscription(Float32, str(self.get_parameter("human_distance_topic").value), self._human_callback, 10)
            self.create_timer(0.2, self._evaluate)
            self.get_logger().info("Canonical monitor is %s; it never publishes /cmd_vel." % ("enabled" if self.enabled else "disabled"))

        def _path_callback(self, message: RosPath) -> None:
            self.latest_path = message

        def _odom_callback(self, message: Odometry) -> None:
            self.latest_odom = message

        def _human_callback(self, message: Float32) -> None:
            self.human_distance = float(message.data) if np.isfinite(message.data) else None

        def _evaluate(self) -> None:
            if not self.enabled or not self.evaluator.config.enabled or self.latest_path is None or self.latest_odom is None:
                return
            pose = self.latest_odom.pose.pose
            twist = self.latest_odom.twist.twist
            context: dict[str, Any] = {
                "path": self.latest_path,
                "speed": float(twist.linear.x),
                "curvature": 0.0,
                "longitudinal_acceleration": 0.0,
                "support_polygon": [(-0.6, -0.5), (0.6, -0.5), (0.6, 0.5), (-0.6, 0.5)],
            }
            if self.human_distance is not None:
                context["human_separation"] = self.human_distance
            report = self.evaluator.evaluate(context)
            self.status_publisher.publish(String(data=json.dumps(report.as_dict(), ensure_ascii=True)))
            if hasattr(report.curvature_speed, "speed_limit") and report.curvature_speed.speed_limit.size:
                self.speed_publisher.publish(Float32(data=float(np.min(report.curvature_speed.speed_limit))))

    rclpy.init()
    node = CanonicalSafetyMonitor()
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
