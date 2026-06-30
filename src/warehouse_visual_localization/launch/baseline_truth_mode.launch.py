import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    baseline_launch = os.path.join(
        bringup_dir,
        "launch",
        "warehouse_nav_lattice_v2.launch.py",
    )
    default_world = os.path.join(
        bringup_dir,
        "worlds",
        "small_warehouse_open_top.world",
    )
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    default_rviz = os.path.join(
        bringup_dir,
        "rviz",
        "forklift_nav_with_cameras.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("auto_generate_map", default_value="true"),
            DeclareLaunchArgument(
                "generated_map_root",
                default_value=os.path.join(
                    os.path.expanduser("~"),
                    "ros2_forklift_warehouse_artifacts",
                    "generated_maps",
                ),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(baseline_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "map": LaunchConfiguration("map"),
                    "auto_generate_map": LaunchConfiguration("auto_generate_map"),
                    "generated_map_root": LaunchConfiguration("generated_map_root"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "mesa_adapter_name": LaunchConfiguration("mesa_adapter_name"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
        ]
    )
