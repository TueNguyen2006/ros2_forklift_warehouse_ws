import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from warehouse_visual_localization.launch_common import (
    get_common_paths,
    select_default_database_path,
)


def generate_launch_description():
    paths = get_common_paths()
    visual_dir = paths["visual_dir"]
    bringup_dir = paths["bringup_dir"]
    ekf_params = os.path.join(visual_dir, "config", "ekf_visual.yaml")

    default_world = os.path.join(bringup_dir, "worlds", "small_warehouse_open_top.world")
    default_rviz = os.path.join(visual_dir, "config", "visual_pose_debug.rviz")
    default_db = select_default_database_path(visual_dir)
    default_eval_csv = os.path.join(
        os.path.expanduser("~"),
        "ros2_forklift_warehouse_artifacts",
        "results",
        "rtabmap_rgbd_eval.csv",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world")
    rviz_mode = LaunchConfiguration("rviz_mode")
    rviz_config = LaunchConfiguration("rviz_config")
    database_path = LaunchConfiguration("database_path")
    localization = LaunchConfiguration("localization")
    pose_source = LaunchConfiguration("pose_source")
    use_wheel_odom_fusion = LaunchConfiguration("use_wheel_odom_fusion")
    drive_model = LaunchConfiguration("drive_model")
    drive_cmd_request_topic = PythonExpression(
        [
            "'/visual_nav/cmd_vel_request' if '",
            use_wheel_odom_fusion,
            "' == 'true' else '/cmd_vel'",
        ]
    )
    drive_cmd_output_topic = PythonExpression(
        [
            "'/rear_steer_controller/reference' if '",
            drive_model,
            "' == 'rear_steer' else '/cmd_vel'",
        ]
    )

    sim_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visual_dir, "launch", "sim_sensors.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gui,
            "rviz": rviz,
            "headless": headless,
            "world": world_file,
            "rviz_mode": rviz_mode,
            "rviz_config": rviz_config,
            "rtabmap_prefix": LaunchConfiguration("rtabmap_prefix"),
            "mesa_adapter_name": LaunchConfiguration("mesa_adapter_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "drive_model": drive_model,
        }.items(),
    )

    rgbd_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visual_dir, "launch", "rgbd_odom.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "publish_tf": PythonExpression(
                ["'false' if '", use_wheel_odom_fusion, "' == 'true' else 'true'"]
            ),
            "odom_topic": PythonExpression(
                ["'/visual_odom' if '", use_wheel_odom_fusion, "' == 'true' else '/odom'"]
            ),
        }.items(),
    )

    ekf_fusion = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(use_wheel_odom_fusion),
        parameters=[
            ekf_params,
            {
                "use_sim_time": use_sim_time,
                "odom0": PythonExpression(
                    [
                        "'/sim_wheel_odom' if '",
                        drive_model,
                        "' == 'planar' else '/rear_steer_controller/odometry'",
                    ]
                ),
                "base_link_frame": PythonExpression(
                    [
                        "'base_footprint' if '",
                        drive_model,
                        "' == 'planar' else 'base_link'",
                    ]
                ),
                "odom0_relative": ParameterValue(
                    PythonExpression(
                        [
                            "'true' if '",
                            drive_model,
                            "' == 'planar' else 'false'",
                        ]
                    ),
                    value_type=bool,
                ),
            },
        ],
        remappings=[("odometry/filtered", "/odom")],
    )

    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visual_dir, "launch", "rtabmap_localization.launch.py")
        ),
        condition=IfCondition(localization),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "database_path": database_path,
            "localization": localization,
            "initial_pose": [
                LaunchConfiguration("spawn_x"),
                " ",
                LaunchConfiguration("spawn_y"),
                " 0 0 0 ",
                LaunchConfiguration("spawn_yaw"),
            ],
        }.items(),
    )

    startup_motion_probe = Node(
        package="warehouse_visual_localization",
        executable="startup_motion_probe.py",
        name="startup_motion_probe",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_startup_motion_probe")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": drive_cmd_request_topic,
                "initial_delay_sec": 2.0,
                "forward_duration_sec": 2.0,
                "reverse_duration_sec": 2.0,
                "linear_speed": 0.08,
                "angular_speed": 0.18,
            }
        ],
    )

    twist_watchdog = Node(
        package="warehouse_visual_localization",
        executable="twist_watchdog.py",
        name="twist_watchdog",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'true' if '",
                    use_wheel_odom_fusion,
                    "' == 'true' and '",
                    drive_model,
                    "' == 'rear_steer' else 'false'",
                ]
            )
        ),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": drive_cmd_request_topic,
                "output_topic": drive_cmd_output_topic,
                "hold_timeout_sec": 0.20,
                "publish_hz": 20.0,
                "use_stamped_output": True,
            }
        ],
    )

    tf_debug = Node(
        package="warehouse_visual_localization",
        executable="tf_debug_node.py",
        name="tf_debug_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tf_debug")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    pose_monitor = Node(
        package="warehouse_visual_localization",
        executable="pose_source_monitor.py",
        name="pose_source_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_pose_source_monitor")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_source": pose_source,
                "consumer_name": "visual_pose",
                "require_map_frame": ParameterValue(localization, value_type=bool),
            }
        ],
    )

    evaluator = Node(
        package="warehouse_visual_localization",
        executable="odom_evaluator.py",
        name="odom_evaluator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_evaluator")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "estimated_odom_topic": "/odom",
                "csv_path": LaunchConfiguration("results_csv"),
                "robot_model_name": "forklift_baseline",
            }
        ],
    )

    scan_retimestamp = Node(
        package="warehouse_visual_localization",
        executable="scan_retimestamp.py",
        name="scan_retimestamp",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_scan_retimestamp")),
        parameters=[
            {
                "input_topic": "/scan",
                "output_topic": "/scan_visual",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("rviz_mode", default_value="visual"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument(
                "rtabmap_prefix",
                default_value=os.path.expanduser("~/ros2_local_overlay/opt/ros/humble"),
            ),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("pose_source", default_value="rgbd_odom_fused"),
            DeclareLaunchArgument("use_wheel_odom_fusion", default_value="true"),
            DeclareLaunchArgument("drive_model", default_value="planar"),
            DeclareLaunchArgument("enable_evaluator", default_value="false"),
            DeclareLaunchArgument("enable_tf_debug", default_value="true"),
            DeclareLaunchArgument("enable_pose_source_monitor", default_value="true"),
            DeclareLaunchArgument("enable_startup_motion_probe", default_value="false"),
            DeclareLaunchArgument("enable_scan_retimestamp", default_value="true"),
            DeclareLaunchArgument("results_csv", default_value=default_eval_csv),
            sim_sensors,
            rgbd_odom,
            ekf_fusion,
            rtabmap_localization,
            twist_watchdog,
            startup_motion_probe,
            scan_retimestamp,
            tf_debug,
            pose_monitor,
            evaluator,
        ]
    )
