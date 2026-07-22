import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    evaluation_dir = get_package_share_directory("forklift_evaluation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "scenario_file",
                default_value=os.path.join(
                    evaluation_dir, "scenarios", "warehouse_route_matrix.yaml"
                ),
            ),
            Node(
                package="forklift_evaluation",
                executable="nav_matrix_runner",
                name="nav_matrix_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "scenario_file": LaunchConfiguration("scenario_file"),
                    }
                ],
            ),
        ]
    )
