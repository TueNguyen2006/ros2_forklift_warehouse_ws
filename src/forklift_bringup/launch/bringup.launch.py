import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    visual_dir = get_package_share_directory("warehouse_visual_localization")
    simulation_dir = get_package_share_directory("forklift_simulation")
    navigation_dir = get_package_share_directory("forklift_navigation")
    launch_path = os.path.join(visual_dir, "launch", "nav_with_estimated_pose.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    simulation_dir, "worlds", "small_warehouse_open_top.world"
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(navigation_dir, "maps", "warehouse_map.yaml"),
            ),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("use_wheel_odom_fusion", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "map": LaunchConfiguration("map"),
                    "localization": LaunchConfiguration("localization"),
                    "use_wheel_odom_fusion": LaunchConfiguration("use_wheel_odom_fusion"),
                }.items(),
            ),
        ]
    )
