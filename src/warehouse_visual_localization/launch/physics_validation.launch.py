"""Opt-in low-speed validation launch; never replaces planar_visual."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from warehouse_visual_localization.launch_common import get_common_paths

def generate_launch_description():
    paths = get_common_paths()
    return LaunchDescription([
        DeclareLaunchArgument("simulation_profile", default_value="physics_validation"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # Phase one reuses the stable sensor stack at a conservative speed.
        # Dynamic contact is intentionally not claimed until the original xacro
        # is decoupled from its fixed world joint and a contact sensor is added.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(paths["visual_dir"], "launch", "nav_with_estimated_pose.launch.py")), launch_arguments={"headless": LaunchConfiguration("headless"), "gui": LaunchConfiguration("gui"), "rviz": LaunchConfiguration("rviz"), "canonical_monitor": "true", "curvature_speed_limit": "true", "rollover_monitor": "true", "benchmark_logger": "true"}.items()),
    ])
