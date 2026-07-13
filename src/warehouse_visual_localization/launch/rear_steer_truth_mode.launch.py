import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
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
            SetLaunchConfiguration(
                "resolved_keepout_mask",
                LaunchConfiguration("keepout_mask").perform(context),
            ),
            SetLaunchConfiguration(
                "resolved_speed_mask",
                LaunchConfiguration("speed_mask").perform(context),
            ),
        ]

    artifacts = ensure_world_map(
        LaunchConfiguration("world").perform(context),
        output_root=LaunchConfiguration("generated_map_root").perform(context),
    )
    return [
        SetLaunchConfiguration("resolved_map", str(artifacts.map_yaml)),
        SetLaunchConfiguration("resolved_keepout_mask", str(artifacts.keepout_yaml)),
        SetLaunchConfiguration("resolved_speed_mask", str(artifacts.speed_yaml)),
        LogInfo(
            msg=(
                "Using generated map assets for rear-steer truth mode: "
                f"map={artifacts.map_yaml} "
                f"keepout={artifacts.keepout_yaml} "
                f"speed={artifacts.speed_yaml}"
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
    default_keepout = os.path.join(bringup_dir, "maps", "warehouse_keepout_mask.yaml")
    default_speed = os.path.join(bringup_dir, "maps", "warehouse_speed_mask.yaml")
    default_nav_params = os.path.join(bringup_dir, "config", "nav2_params_realistic.yaml")
    default_collision_monitor = os.path.join(
        bringup_dir, "config", "collision_monitor_smoke.yaml"
    )
    default_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    realistic_controller_config = os.path.join(
        realistic_dir, "config", "rear_steer_controller.yaml"
    )
    robot_description = build_visual_rear_steer_robot_description(
        realistic_dir, realistic_controller_config, lock_forks=True
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")
    effective_gui = PythonExpression(
        ["'false' if '", headless, "' == 'true' else '", gui, "'"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "gui": effective_gui,
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
                "use_sim_time": use_sim_time,
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rear_steer_drive_controller = Node(
        package="warehouse_visual_localization",
        executable="rear_steer_drive_controller.py",
        name="rear_steer_drive_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_topic": "/realistic_nav/cmd_vel_request",
                "traction_command_topic": "/traction_velocity_controller/commands",
                "steering_command_topic": "/rear_steering_position_controller/commands",
            }
        ],
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "forklift_nav_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "params_file": LaunchConfiguration("params_file"),
            "collision_monitor_file": LaunchConfiguration("collision_monitor_file"),
            "map": LaunchConfiguration("resolved_map"),
            "keepout_mask": LaunchConfiguration("resolved_keepout_mask"),
            "speed_mask": LaunchConfiguration("resolved_speed_mask"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "use_amcl": "false",
            "use_costmap_filters": LaunchConfiguration("use_costmap_filters"),
            "profile_config": LaunchConfiguration("profile_config"),
            "load_profile": LaunchConfiguration("load_profile"),
            "use_stability_guard": LaunchConfiguration("use_stability_guard"),
            "use_collision_monitor": LaunchConfiguration("use_collision_monitor"),
            "cmd_vel_out_topic": "/realistic_nav/cmd_vel_request",
            "velocity_smoother_passthrough_topic": "/realistic_nav/cmd_vel_request",
            "publish_map_to_odom_tf": "true",
            "map_to_odom_x": LaunchConfiguration("spawn_x"),
            "map_to_odom_y": LaunchConfiguration("spawn_y"),
            "map_to_odom_z": "0.0",
            "map_to_odom_roll": "0.0",
            "map_to_odom_pitch": "0.0",
            "map_to_odom_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    gazebo_goal_bridge = Node(
        package="warehouse_visual_localization",
        executable="gazebo_goal_bridge.py",
        name="gazebo_goal_bridge",
        output="screen",
        parameters=[
            {
                "goal_topic": "/gazebo/nav_goal_pose",
                "nav_action_name": "/navigate_to_pose",
                "goal_frame": "map",
                "robot_frame": "base_footprint",
                "use_current_yaw": False,
                "publish_goal_pose_topic": True,
                "goal_pose_topic": "/goal_pose",
            }
        ],
    )

    nav_debug_logger = Node(
        package="forklift_nav_bringup",
        executable="nav_debug_logger",
        name="nav_debug_logger",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "log_dir": os.path.join(
                    os.path.expanduser("~"), ".ros", "forklift_nav_logs"
                ),
                "log_prefix": "realistic_nav",
                "global_frame": "map",
                "odom_frame": "odom",
                "robot_frame": "base_footprint",
                "cmd_vel_topic": "/realistic_nav/cmd_vel_request",
                "raw_cmd_vel_topic": "cmd_vel_nav",
            }
        ],
    )

    pose_monitor = Node(
        package="warehouse_visual_localization",
        executable="pose_source_monitor.py",
        name="pose_source_monitor_realistic_truth",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_source": "ground_truth_static_map_tf",
                "consumer_name": "rear_steer_truth_mode",
                "require_map_frame": True,
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
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("keepout_mask", default_value=default_keepout),
            DeclareLaunchArgument("speed_mask", default_value=default_speed),
            DeclareLaunchArgument("auto_generate_map", default_value="true"),
            DeclareLaunchArgument(
                "generated_map_root",
                default_value=str(default_output_root()),
            ),
            DeclareLaunchArgument("params_file", default_value=default_nav_params),
            DeclareLaunchArgument(
                "collision_monitor_file", default_value=default_collision_monitor
            ),
            DeclareLaunchArgument(
                "profile_config",
                default_value=os.path.join(
                    visual_dir,
                    "config",
                    "stability_profiles_visual.yaml",
                ),
            ),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
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
            DeclareLaunchArgument("use_costmap_filters", default_value="false"),
            DeclareLaunchArgument("use_stability_guard", default_value="false"),
            DeclareLaunchArgument("use_collision_monitor", default_value="false"),
            LogInfo(
                msg=(
                    "Launching rear-steer truth mode: Nav2 will consume a static map->odom "
                    "truth TF for baseline comparison, not a learned/global localization source."
                )
            ),
            OpaqueFunction(function=_configure_runtime_map),
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
                    on_exit=[rear_steer_drive_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[
                                nav_stack,
                                gazebo_goal_bridge,
                                nav_debug_logger,
                                pose_monitor,
                            ],
                        )
                    ],
                )
            ),
            rviz_node,
        ]
    )
