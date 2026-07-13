import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from forklift_nav_bringup.world_map_generator import default_output_root, ensure_world_map

from warehouse_visual_localization.launch_common import (
    build_visual_rear_steer_robot_description,
    get_common_paths,
    make_runtime_env_actions,
)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _configure_runtime_map(context, *_, **__):
    if not _as_bool(LaunchConfiguration("auto_generate_map").perform(context)):
        return [
            SetLaunchConfiguration(
                "resolved_map",
                LaunchConfiguration("map").perform(context),
            ),
        ]

    artifacts = ensure_world_map(
        LaunchConfiguration("world").perform(context),
        output_root=LaunchConfiguration("generated_map_root").perform(context),
    )
    return [
        SetLaunchConfiguration("resolved_map", str(artifacts.map_yaml)),
        LogInfo(
            msg=(
                "Rear-steer sim-only mode using generated map assets for visualization only: "
                f"map={artifacts.map_yaml}"
            )
        ),
    ]


def generate_launch_description():
    paths = get_common_paths()
    visual_dir = paths["visual_dir"]
    bringup_dir = paths["bringup_dir"]
    gazebo_ros_dir = paths["gazebo_ros_dir"]
    realistic_dir = paths["realistic_dir"]
    ros_gazebo_plugin_dir = paths["ros_gazebo_plugin_dir"]

    default_world = os.path.join(bringup_dir, "worlds", "small_warehouse_open_top.world")
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    default_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    realistic_controller_config = os.path.join(
        realistic_dir, "config", "rear_steer_controller.yaml"
    )
    robot_description = build_visual_rear_steer_robot_description(
        realistic_dir, realistic_controller_config, lock_forks=True
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": PythonExpression(
                [
                    "'false' if '",
                    LaunchConfiguration("headless"),
                    "' == 'true' else '",
                    LaunchConfiguration("gui"),
                    "'",
                ]
            ),
            "verbose": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "forklift_baseline",
            "-topic",
            "robot_description",
            "-timeout",
            "120.0",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    traction_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "traction_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120.0",
            "--service-call-timeout",
            "120.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    rear_steering_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_steering_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120.0",
            "--service-call-timeout",
            "120.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120.0",
            "--service-call-timeout",
            "120.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    rear_steer_drive_controller = Node(
        package="warehouse_visual_localization",
        executable="rear_steer_drive_controller.py",
        name="rear_steer_drive_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "cmd_topic": "/realistic_nav/cmd_vel_request",
                "traction_command_topic": "/traction_velocity_controller/commands",
                "steering_command_topic": "/rear_steering_position_controller/commands",
            }
        ],
    )

    pose_monitor = Node(
        package="warehouse_visual_localization",
        executable="pose_source_monitor.py",
        name="pose_source_monitor_realistic_sim_only",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "pose_source": "rear_steer_joint_odometry",
                "consumer_name": "rear_steer_sim_only",
                "track_map_frame": False,
                "require_map_frame": False,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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
                default_value=str(default_output_root()),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
            DeclareLaunchArgument(
                "rtabmap_prefix",
                default_value=os.path.expanduser("~/ros2_local_overlay/opt/ros/humble"),
            ),
            DeclareLaunchArgument("force_software_rendering", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            OpaqueFunction(function=_configure_runtime_map),
            LogInfo(
                msg=(
                    "Launching rear-steer sim-only mode: no Nav2, no static map->odom truth TF, "
                    "use this mode to debug drivetrain, physics, and sensors."
                )
            ),
            *make_runtime_env_actions(bringup_dir, ros_gazebo_plugin_dir),
            robot_state_publisher,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[traction_velocity_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=traction_velocity_controller,
                    on_exit=[rear_steering_position_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rear_steering_position_controller,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[rear_steer_drive_controller, pose_monitor],
                )
            ),
            rviz_node,
        ]
    )
