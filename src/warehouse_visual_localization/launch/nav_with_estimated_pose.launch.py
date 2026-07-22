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
from launch_ros.parameter_descriptions import ParameterValue
from forklift_simulation.world_map_generator import default_output_root, ensure_world_map

from warehouse_visual_localization.launch_common import (
    get_common_paths,
    select_default_database_path,
)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _configure_runtime_map(context, *_, **__):
    if not _as_bool(LaunchConfiguration("auto_generate_map").perform(context)):
        return [
            SetLaunchConfiguration(
                "resolved_map",
                LaunchConfiguration("map").perform(context),
            )
        ]

    artifacts = ensure_world_map(
        LaunchConfiguration("world").perform(context),
        output_root=LaunchConfiguration("generated_map_root").perform(context),
    )
    return [
        SetLaunchConfiguration("resolved_map", str(artifacts.map_yaml)),
        LogInfo(msg=f"Using generated map for visual navigation: {artifacts.map_yaml}"),
    ]


def generate_launch_description():
    paths = get_common_paths()
    visual_dir = paths["visual_dir"]
    navigation_dir = paths["navigation_dir"]
    simulation_dir = paths["simulation_dir"]

    default_map = os.path.join(navigation_dir, "maps", "warehouse_map.yaml")
    default_world = os.path.join(simulation_dir, "worlds", "small_warehouse_open_top.world")
    default_nav_params = os.path.join(visual_dir, "config", "nav2_params_visual.yaml")
    default_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    default_db = select_default_database_path(visual_dir)
    default_collision_monitor = os.path.join(
        visual_dir, "config", "collision_monitor_visual.yaml"
    )
    default_stability_profiles = os.path.join(
        visual_dir, "config", "stability_profiles_visual.yaml"
    )
    default_eval_csv = os.path.join(
        os.path.expanduser("~"),
        "ros2_forklift_warehouse_artifacts",
        "results",
        "rtabmap_rgbd_eval.csv",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    pose_source = LaunchConfiguration("pose_source")
    use_wheel_odom_fusion = LaunchConfiguration("use_wheel_odom_fusion")
    canonical_filter_enabled = PythonExpression([
        "'true' if ('", LaunchConfiguration("curvature_speed_limit"), "' == 'true' or '",
        LaunchConfiguration("human_ssm"), "' == 'true' or '", LaunchConfiguration("cbf_filter"),
        "' == 'true') else 'false'",
    ])
    canonical_monitor_enabled = PythonExpression([
        "'true' if ('", LaunchConfiguration("canonical_monitor"), "' == 'true' or '",
        LaunchConfiguration("curvature_speed_limit"), "' == 'true' or '",
        LaunchConfiguration("corridor_monitor"), "' == 'true' or '",
        LaunchConfiguration("rollover_monitor"), "' == 'true' or '",
        LaunchConfiguration("human_ssm"), "' == 'true') else 'false'",
    ])

    visual_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visual_dir, "launch", "visual_pose.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "headless": LaunchConfiguration("headless"),
            "world": LaunchConfiguration("world"),
            "rviz_mode": "visual",
            "rviz_config": LaunchConfiguration("rviz_config"),
            "rtabmap_prefix": LaunchConfiguration("rtabmap_prefix"),
            "mesa_adapter_name": LaunchConfiguration("mesa_adapter_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "database_path": LaunchConfiguration("database_path"),
            "localization": localization,
            "pose_source": pose_source,
            "use_wheel_odom_fusion": use_wheel_odom_fusion,
            "enable_evaluator": LaunchConfiguration("enable_evaluator"),
            "enable_tf_debug": LaunchConfiguration("enable_tf_debug"),
            "enable_pose_source_monitor": "false",
            "enable_startup_motion_probe": LaunchConfiguration(
                "enable_startup_motion_probe"
            ),
            "results_csv": LaunchConfiguration("results_csv"),
        }.items(),
    )

    wait_for_map_tf = Node(
        package="warehouse_visual_localization",
        executable="wait_for_map_tf.py",
        name="wait_for_map_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame": "map",
                "odom_frame": "odom",
                "robot_frame": "base_footprint",
                "require_global_frame": ParameterValue(
                    localization, value_type=bool
                ),
                "timeout_sec": LaunchConfiguration("wait_for_map_tf_timeout"),
            }
        ],
    )

    def create_nav_stack(*, publish_map_to_odom_tf: str, condition=None):
        return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, "launch", "navigation.launch.py")
            ),
            condition=condition,
            launch_arguments={
                "use_sim_time": use_sim_time,
                "rviz": "false",
                "params_file": LaunchConfiguration("params_file"),
                "collision_monitor_file": LaunchConfiguration("collision_monitor_file"),
                "map": LaunchConfiguration("resolved_map"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "use_amcl": "false",
                "use_costmap_filters": "false",
                "profile_config": LaunchConfiguration("profile_config"),
                "load_profile": LaunchConfiguration("load_profile"),
                "use_stability_guard": LaunchConfiguration("use_stability_guard"),
                "use_collision_monitor": LaunchConfiguration("use_collision_monitor"),
                "cmd_vel_out_topic": "/visual_nav/cmd_vel_request",
                "velocity_smoother_passthrough_topic": "/visual_nav/cmd_vel_request",
                "publish_map_to_odom_tf": publish_map_to_odom_tf,
                "map_to_odom_x": LaunchConfiguration("spawn_x"),
                "map_to_odom_y": LaunchConfiguration("spawn_y"),
                "map_to_odom_z": "0.0",
                "map_to_odom_roll": "0.0",
                "map_to_odom_pitch": "0.0",
                "map_to_odom_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        )

    nav_stack_with_localization = create_nav_stack(publish_map_to_odom_tf="false")
    nav_stack_without_localization = create_nav_stack(
        publish_map_to_odom_tf="true",
        condition=IfCondition(
            PythonExpression(
                [
                    "'true' if '",
                    localization,
                    "' != 'true' else 'false'",
                ]
            )
        ),
    )

    def create_nav_debug_logger(condition=None):
        return Node(
            package="forklift_navigation",
            executable="nav_debug_logger",
            name="nav_debug_logger",
            output="screen",
            condition=condition,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "log_dir": os.path.join(
                        os.path.expanduser("~"), ".ros", "forklift_nav_logs"
                    ),
                    "log_prefix": "visual_nav",
                    "global_frame": "map",
                    "odom_frame": "odom",
                    "robot_frame": "base_footprint",
                    "cmd_vel_topic": "/visual_nav/cmd_vel_request",
                    "raw_cmd_vel_topic": "cmd_vel_nav",
                }
            ],
        )

    def create_gazebo_goal_bridge(condition=None):
        return Node(
            package="warehouse_visual_localization",
            executable="gazebo_goal_bridge.py",
            name="gazebo_goal_bridge",
            output="screen",
            condition=condition,
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

    def create_pose_monitor(condition=None):
        return Node(
            package="warehouse_visual_localization",
            executable="pose_source_monitor.py",
            name="pose_source_monitor_nav",
            output="screen",
            condition=condition,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "pose_source": pose_source,
                    "consumer_name": "nav_with_estimated_pose",
                    "require_map_frame": ParameterValue(localization, value_type=bool),
                    "required_odom_topics_csv": PythonExpression(
                        [
                            "'/visual_odom' if '",
                            use_wheel_odom_fusion,
                            "' == 'true' else '/odom'",
                        ]
                    ),
                }
            ],
        )

    nav_debug_logger = create_nav_debug_logger()
    gazebo_goal_bridge = create_gazebo_goal_bridge()
    pose_monitor = create_pose_monitor()
    canonical_monitor = Node(
        package="warehouse_visual_localization",
        executable="canonical_trajectory_monitor.py",
        name="canonical_trajectory_monitor",
        output="screen",
        condition=IfCondition(canonical_monitor_enabled),
        parameters=[{
            "enabled": True,
            "curvature_speed_limit": ParameterValue(LaunchConfiguration("curvature_speed_limit"), value_type=bool),
            "corridor_monitor": ParameterValue(LaunchConfiguration("corridor_monitor"), value_type=bool),
            "rollover_monitor": ParameterValue(LaunchConfiguration("rollover_monitor"), value_type=bool),
            "human_ssm": ParameterValue(LaunchConfiguration("human_ssm"), value_type=bool),
            "config_path": os.path.join(visual_dir, "config", "core", "canonical_constraints.yaml"),
            "use_sim_time": use_sim_time,
        }],
    )
    canonical_filter = Node(
        package="warehouse_visual_localization",
        executable="canonical_command_filter.py",
        name="canonical_command_filter",
        output="screen",
        condition=IfCondition(canonical_filter_enabled),
        parameters=[{
            "enabled": True,
            "cbf_filter": ParameterValue(LaunchConfiguration("cbf_filter"), value_type=bool),
            "input_topic": "/visual_nav/cmd_vel_request",
            "output_topic": "/canonical/cmd_vel_filtered",
            "max_speed": 0.32,
            "max_yaw_rate": 0.32,
        }],
    )
    canonical_human_source = Node(
        package="warehouse_visual_localization",
        executable="canonical_human_distance_source.py",
        name="canonical_human_distance_source",
        output="screen",
        condition=IfCondition(LaunchConfiguration("human_ssm")),
        parameters=[{"enabled": True, "use_sim_time": use_sim_time}],
    )
    canonical_benchmark_logger = Node(
        package="warehouse_visual_localization", executable="canonical_benchmark_logger.py",
        name="canonical_benchmark_logger", output="screen",
        condition=IfCondition(LaunchConfiguration("benchmark_logger")),
        parameters=[{"enabled": True, "use_sim_time": use_sim_time}],
    )
    planar_motion_guard = Node(
        package="warehouse_visual_localization",
        executable="planar_motion_guard.py",
        name="planar_motion_guard",
        output="screen",
        parameters=[
            {
                "input_topic": PythonExpression(["'/canonical/cmd_vel_filtered' if '", canonical_filter_enabled, "' == 'true' else '/visual_nav/cmd_vel_request'"]),
                "output_topic": "/cmd_vel",
                "hold_timeout_sec": 0.25,
                "publish_hz": 20.0,
                "linear_deadband": 0.02,
                "min_turn_linear_speed": 0.08,
                "turn_command_threshold": 0.20,
                "max_angular_speed": 0.32,
                "max_angular_speed_at_low_linear": 0.20,
                "default_linear_sign": 1.0,
            }
        ],
    )

    nav_debug_logger_without_localization = create_nav_debug_logger(
        condition=IfCondition(
            PythonExpression(
                [
                    "'true' if '",
                    localization,
                    "' != 'true' else 'false'",
                ]
            )
        )
    )
    gazebo_goal_bridge_without_localization = create_gazebo_goal_bridge(
        condition=IfCondition(
            PythonExpression(
                [
                    "'true' if '",
                    localization,
                    "' != 'true' else 'false'",
                ]
            )
        )
    )
    pose_monitor_without_localization = create_pose_monitor(
        condition=IfCondition(
            PythonExpression(
                [
                    "'true' if '",
                    localization,
                    "' != 'true' else 'false'",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("auto_generate_map", default_value="false"),
            DeclareLaunchArgument(
                "generated_map_root",
                default_value=str(default_output_root()),
            ),
            DeclareLaunchArgument("params_file", default_value=default_nav_params),
            DeclareLaunchArgument(
                "collision_monitor_file", default_value=default_collision_monitor
            ),
            DeclareLaunchArgument(
                "profile_config", default_value=default_stability_profiles
            ),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("pose_source", default_value="rgbd_odom_fused"),
            DeclareLaunchArgument("use_wheel_odom_fusion", default_value="true"),
            DeclareLaunchArgument("use_stability_guard", default_value="false"),
            DeclareLaunchArgument("use_collision_monitor", default_value="false"),
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
            DeclareLaunchArgument("enable_gazebo_goal_bridge", default_value="true"),
            DeclareLaunchArgument("enable_pose_source_monitor", default_value="true"),
            DeclareLaunchArgument("enable_startup_motion_probe", default_value="false"),
            DeclareLaunchArgument("canonical_monitor", default_value="false"),
            DeclareLaunchArgument("curvature_speed_limit", default_value="false"),
            DeclareLaunchArgument("corridor_monitor", default_value="false"),
            DeclareLaunchArgument("rollover_monitor", default_value="false"),
            DeclareLaunchArgument("human_ssm", default_value="false"),
            DeclareLaunchArgument("cbf_filter", default_value="false"),
            DeclareLaunchArgument("benchmark_logger", default_value="false"),
            DeclareLaunchArgument("wait_for_map_tf_timeout", default_value="90.0"),
            DeclareLaunchArgument("results_csv", default_value=default_eval_csv),
            OpaqueFunction(function=_configure_runtime_map),
            visual_pose,
            LogInfo(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'true' if '",
                            localization,
                            "' != 'true' else 'false'",
                        ]
                    )
                ),
                msg=(
                    "Localization is disabled. Launching Nav2 in odom-only visual mode "
                    "with static map->odom for manual testing."
                ),
            ),
            wait_for_map_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_map_tf,
                    on_exit=[
                        nav_stack_without_localization,
                        planar_motion_guard,
                        canonical_monitor,
                        canonical_filter,
                        canonical_human_source,
                        canonical_benchmark_logger,
                        nav_debug_logger_without_localization,
                        gazebo_goal_bridge_without_localization,
                        pose_monitor_without_localization,
                    ],
                ),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'true' if '",
                            localization,
                            "' != 'true' else 'false'",
                        ]
                    )
                ),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_map_tf,
                    on_exit=[
                        nav_stack_with_localization,
                        planar_motion_guard,
                        canonical_monitor,
                        canonical_filter,
                        canonical_human_source,
                        canonical_benchmark_logger,
                        nav_debug_logger,
                        gazebo_goal_bridge,
                        pose_monitor,
                    ],
                ),
                condition=IfCondition(localization),
            ),
        ]
    )
