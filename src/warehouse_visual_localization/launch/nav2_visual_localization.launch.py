import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    visual_dir = get_package_share_directory("warehouse_visual_localization")
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    wrapper_launch = os.path.join(visual_dir, "launch", "visual_estimated_mode.launch.py")
    default_world = os.path.join(bringup_dir, "worlds", "small_warehouse_open_top.world")
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    default_params = os.path.join(visual_dir, "config", "nav2_params_visual.yaml")
    default_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    default_db = os.path.join(
        os.path.expanduser("~"),
        "ros2_forklift_warehouse_artifacts",
        "results",
        "test_mapping.db",
    )
    default_results_csv = os.path.join(
        os.path.expanduser("~"),
        "ros2_forklift_warehouse_artifacts",
        "results",
        "rtabmap_rgbd_eval.csv",
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
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument(
                "rtabmap_prefix",
                default_value=os.path.expanduser("~/ros2_local_overlay/opt/ros/humble"),
            ),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("enable_evaluator", default_value="false"),
            DeclareLaunchArgument("enable_tf_debug", default_value="true"),
            DeclareLaunchArgument("enable_nav_debug_logger", default_value="true"),
            DeclareLaunchArgument("enable_pose_source_monitor", default_value="true"),
            DeclareLaunchArgument("enable_startup_motion_probe", default_value="false"),
            DeclareLaunchArgument("use_wheel_odom_fusion", default_value="true"),
            DeclareLaunchArgument("wait_for_map_tf_timeout", default_value="90.0"),
            DeclareLaunchArgument("results_csv", default_value=default_results_csv),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(wrapper_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "map": LaunchConfiguration("map"),
                    "auto_generate_map": LaunchConfiguration("auto_generate_map"),
                    "generated_map_root": LaunchConfiguration("generated_map_root"),
                    "params_file": LaunchConfiguration("params_file"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "database_path": LaunchConfiguration("database_path"),
                    "rtabmap_prefix": LaunchConfiguration("rtabmap_prefix"),
                    "mesa_adapter_name": LaunchConfiguration("mesa_adapter_name"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "enable_evaluator": LaunchConfiguration("enable_evaluator"),
                    "enable_tf_debug": LaunchConfiguration("enable_tf_debug"),
                    "enable_nav_debug_logger": LaunchConfiguration(
                        "enable_nav_debug_logger"
                    ),
                    "enable_pose_source_monitor": LaunchConfiguration(
                        "enable_pose_source_monitor"
                    ),
                    "enable_startup_motion_probe": LaunchConfiguration(
                        "enable_startup_motion_probe"
                    ),
                    "use_wheel_odom_fusion": LaunchConfiguration(
                        "use_wheel_odom_fusion"
                    ),
                    "wait_for_map_tf_timeout": LaunchConfiguration(
                        "wait_for_map_tf_timeout"
                    ),
                    "results_csv": LaunchConfiguration("results_csv"),
                }.items(),
            )
        ]
    )
