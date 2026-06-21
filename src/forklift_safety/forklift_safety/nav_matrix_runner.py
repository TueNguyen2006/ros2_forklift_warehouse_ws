import json
import math
import time
from pathlib import Path

import rclpy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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


def result_to_text(result) -> str:
    if result == TaskResult.SUCCEEDED:
        return "SUCCEEDED"
    if result == TaskResult.CANCELED:
        return "CANCELED"
    if result == TaskResult.FAILED:
        return "FAILED"
    return "UNKNOWN"


def main(args=None) -> None:
    rclpy.init(args=args)

    config_node = rclpy.create_node("nav_matrix_runner_config")
    config_node.declare_parameter("scenario_file", "")
    config_node.declare_parameter("result_file", "")
    config_node.declare_parameter("goal_timeout_sec", 180.0)

    scenario_file = config_node.get_parameter("scenario_file").value
    result_file = config_node.get_parameter("result_file").value
    goal_timeout_sec = float(config_node.get_parameter("goal_timeout_sec").value)

    if not scenario_file:
        raise RuntimeError("scenario_file parameter is required.")

    with Path(scenario_file).open("r", encoding="utf-8") as handle:
        scenario = yaml.safe_load(handle) or {}

    navigator = BasicNavigator()

    initial = scenario["initial_pose"]
    initial_pose = make_pose(initial["x"], initial["y"], initial["yaw"])
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active(localizer="amcl")

    summary = {
        "scenario_file": scenario_file,
        "started_at": time.time(),
        "goals": [],
    }

    for goal in scenario.get("goals", []):
        pose = make_pose(goal["x"], goal["y"], goal["yaw"])
        pose.header.stamp = navigator.get_clock().now().to_msg()

        navigator.get_logger().info(
            f"Navigating to {goal['name']} at ({goal['x']:.2f}, {goal['y']:.2f})"
        )
        start_time = time.monotonic()
        navigator.goToPose(pose)

        timed_out = False
        while not navigator.isTaskComplete():
            time.sleep(0.2)
            if time.monotonic() - start_time > goal_timeout_sec:
                navigator.get_logger().warning(
                    f"Timeout on goal {goal['name']}, canceling."
                )
                navigator.cancelTask()
                timed_out = True
                break

        result = navigator.getResult()
        summary["goals"].append(
            {
                "name": goal["name"],
                "x": goal["x"],
                "y": goal["y"],
                "yaw": goal["yaw"],
                "status": result_to_text(result),
                "timed_out": timed_out,
                "elapsed_sec": round(time.monotonic() - start_time, 3),
            }
        )

    summary["finished_at"] = time.time()
    summary["success_count"] = sum(1 for item in summary["goals"] if item["status"] == "SUCCEEDED")
    summary["failure_count"] = len(summary["goals"]) - summary["success_count"]

    output_path = Path(result_file) if result_file else Path(scenario_file).with_suffix(".results.json")
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    navigator.get_logger().info(f"Route matrix summary written to {output_path}")
    navigator.destroy_node()
    config_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
