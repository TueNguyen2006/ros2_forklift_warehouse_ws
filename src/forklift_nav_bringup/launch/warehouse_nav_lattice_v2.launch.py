import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    baseline_launch = os.path.join(
        bringup_dir,
        "launch",
        "warehouse_nav_baseline.launch.py",
    )
    default_params = os.path.join(
        bringup_dir,
        "config",
        "nav2_params_v2_lattice.yaml",
    )
    default_world = os.path.join(
        bringup_dir,
        "worlds",
        "small_warehouse_open_top.world",
    )
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    effective_gui = PythonExpression(
        [
            "'false' if '",
            LaunchConfiguration("headless"),
            "' == 'true' else '",
            LaunchConfiguration("gui"),
            "'",
        ]
    )
    effective_rviz = PythonExpression(
        [
            "'false' if '",
            LaunchConfiguration("headless"),
            "' == 'true' else '",
            LaunchConfiguration("rviz"),
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            # Keep supporting the more intuitive alias used in manual testing.
            # headless:=true disables both the Gazebo GUI and RViz.
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("use_amcl", default_value="false"),
            DeclareLaunchArgument("use_initial_pose_publisher", default_value="false"),
            DeclareLaunchArgument("use_costmap_filters", default_value="false"),
            DeclareLaunchArgument("use_stability_guard", default_value="false"),
            DeclareLaunchArgument("use_collision_monitor", default_value="false"),
            DeclareLaunchArgument("enable_debug_logger", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(baseline_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": effective_gui,
                    "rviz": effective_rviz,
                    "load_profile": LaunchConfiguration("load_profile"),
                    "world": LaunchConfiguration("world"),
                    "map": LaunchConfiguration("map"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "use_amcl": LaunchConfiguration("use_amcl"),
                    "use_initial_pose_publisher": LaunchConfiguration(
                        "use_initial_pose_publisher"
                    ),
                    "use_costmap_filters": LaunchConfiguration("use_costmap_filters"),
                    "use_stability_guard": LaunchConfiguration("use_stability_guard"),
                    "use_collision_monitor": LaunchConfiguration(
                        "use_collision_monitor"
                    ),
                    "enable_debug_logger": LaunchConfiguration(
                        "enable_debug_logger"
                    ),
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),
        ]
    )
