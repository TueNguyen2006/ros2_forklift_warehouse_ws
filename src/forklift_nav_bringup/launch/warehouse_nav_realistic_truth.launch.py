import os

from ament_index_python.packages import get_package_share_directory

from forklift_nav_bringup.world_map_generator import default_output_root
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    visual_dir = get_package_share_directory("warehouse_visual_localization")
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    default_world = os.path.join(bringup_dir, "worlds", "small_warehouse_open_top.world")
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    default_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    realistic_launch = os.path.join(
        visual_dir,
        "launch",
        "rear_steer_truth_mode.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("auto_generate_map", default_value="true"),
            DeclareLaunchArgument(
                "generated_map_root",
                default_value=str(default_output_root()),
            ),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("use_amcl", default_value="false"),
            DeclareLaunchArgument("use_costmap_filters", default_value="false"),
            DeclareLaunchArgument("use_stability_guard", default_value="false"),
            DeclareLaunchArgument("use_collision_monitor", default_value="false"),
            DeclareLaunchArgument("enable_debug_logger", default_value="false"),
            LogInfo(
                msg=(
                    "Launching rear-steer truth-nav mode: Nav2 will receive a static truth "
                    "map->odom TF for baseline comparison/debug."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realistic_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "map": LaunchConfiguration("map"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "auto_generate_map": LaunchConfiguration("auto_generate_map"),
                    "generated_map_root": LaunchConfiguration("generated_map_root"),
                    "load_profile": LaunchConfiguration("load_profile"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "use_costmap_filters": LaunchConfiguration("use_costmap_filters"),
                    "use_stability_guard": LaunchConfiguration("use_stability_guard"),
                    "use_collision_monitor": LaunchConfiguration(
                        "use_collision_monitor"
                    ),
                }.items(),
            ),
        ]
    )
