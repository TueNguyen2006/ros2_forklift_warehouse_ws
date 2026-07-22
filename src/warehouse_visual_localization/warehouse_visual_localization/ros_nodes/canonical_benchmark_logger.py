"""Opt-in ROS-topic logger for canonical Gazebo benchmark artifacts."""
from __future__ import annotations

import csv
from pathlib import Path


def main() -> None:
    import rclpy
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.node import Node
    from std_msgs.msg import Float32

    class BenchmarkLogger(Node):
        def __init__(self) -> None:
            super().__init__("canonical_benchmark_logger")
            self.declare_parameter("enabled", False)
            self.declare_parameter("output_dir", str(Path.home() / "ros2_forklift_warehouse_artifacts" / "canonical_benchmarks" / "gazebo_latest"))
            self.enabled = bool(self.get_parameter("enabled").value)
            self.rows: list[dict[str, float]] = []
            self.plan_length = 0.0
            self.limit = float("nan")
            self.roll = float("nan")
            self.margin = float("nan")
            self.create_subscription(Odometry, "/odom", self._odom, 20)
            self.create_subscription(RosPath, "/plan", self._plan, 10)
            self.create_subscription(Float32, "/canonical/reference_speed_limit", lambda message: setattr(self, "limit", float(message.data)), 10)
            self.create_subscription(Float32, "/canonical/roll_utilization", lambda message: setattr(self, "roll", float(message.data)), 10)
            self.create_subscription(Float32, "/canonical/corridor_margin", lambda message: setattr(self, "margin", float(message.data)), 10)

        def _plan(self, message: RosPath) -> None:
            self.plan_length = sum(((b.pose.position.x-a.pose.position.x)**2 + (b.pose.position.y-a.pose.position.y)**2) ** 0.5 for a, b in zip(message.poses, message.poses[1:]))

        def _odom(self, message: Odometry) -> None:
            if not self.enabled:
                return
            self.rows.append({"time_s": self.get_clock().now().nanoseconds / 1e9, "speed_mps": message.twist.twist.linear.x, "yaw_rate_rps": message.twist.twist.angular.z, "reference_speed_limit_mps": self.limit, "roll_utilization": self.roll, "corridor_margin_m": self.margin, "path_length_m": self.plan_length})

        def close(self) -> None:
            if not self.rows:
                return
            output = Path(str(self.get_parameter("output_dir").value)); output.mkdir(parents=True, exist_ok=True)
            with (output / "timeseries.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0])); writer.writeheader(); writer.writerows(self.rows)

    rclpy.init(); node = BenchmarkLogger()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
