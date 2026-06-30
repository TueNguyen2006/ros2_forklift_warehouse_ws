import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    safety_dir = get_package_share_directory("forklift_safety")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("rviz")
    params_file = LaunchConfiguration("params_file")
    collision_monitor_file = LaunchConfiguration("collision_monitor_file")
    map_file = LaunchConfiguration("map")
    keepout_mask_file = LaunchConfiguration("keepout_mask")
    speed_mask_file = LaunchConfiguration("speed_mask")
    profile_config = LaunchConfiguration("profile_config")
    load_profile = LaunchConfiguration("load_profile")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    velocity_smoother_input_topic = LaunchConfiguration(
        "velocity_smoother_input_topic"
    )
    velocity_smoother_output_topic = LaunchConfiguration(
        "velocity_smoother_output_topic"
    )
    velocity_smoother_passthrough_topic = LaunchConfiguration(
        "velocity_smoother_passthrough_topic"
    )
    stability_guard_output_topic = LaunchConfiguration(
        "stability_guard_output_topic"
    )
    rviz_config = LaunchConfiguration("rviz_config")
    bt_nav_to_pose = LaunchConfiguration("bt_nav_to_pose")
    bt_nav_through_poses = LaunchConfiguration("bt_nav_through_poses")
    use_amcl = LaunchConfiguration("use_amcl")
    use_costmap_filters = LaunchConfiguration("use_costmap_filters")
    use_stability_guard = LaunchConfiguration("use_stability_guard")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_z = LaunchConfiguration("map_to_odom_z")
    map_to_odom_roll = LaunchConfiguration("map_to_odom_roll")
    map_to_odom_pitch = LaunchConfiguration("map_to_odom_pitch")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_amcl", default_value="false"),
            DeclareLaunchArgument("use_costmap_filters", default_value="false"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument("map_to_odom_x", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_y", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_z", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_roll", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_pitch", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "collision_monitor_file",
                default_value=os.path.join(bringup_dir, "config", "collision_monitor.yaml"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(bringup_dir, "maps", "warehouse_map.yaml"),
            ),
            DeclareLaunchArgument(
                "keepout_mask",
                default_value=os.path.join(bringup_dir, "maps", "warehouse_keepout_mask.yaml"),
            ),
            DeclareLaunchArgument(
                "speed_mask",
                default_value=os.path.join(bringup_dir, "maps", "warehouse_speed_mask.yaml"),
            ),
            DeclareLaunchArgument(
                "profile_config",
                default_value=os.path.join(
                    safety_dir, "config", "stability_profiles.yaml"
                ),
            ),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("use_stability_guard", default_value="true"),
            DeclareLaunchArgument("use_collision_monitor", default_value="true"),
            DeclareLaunchArgument("cmd_vel_in_topic", default_value="cmd_vel_stability"),
            DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "velocity_smoother_input_topic",
                default_value="cmd_vel_nav",
            ),
            DeclareLaunchArgument(
                "velocity_smoother_output_topic",
                default_value="cmd_vel_smoothed",
            ),
            DeclareLaunchArgument(
                "velocity_smoother_passthrough_topic",
                default_value="/cmd_vel",
            ),
            DeclareLaunchArgument(
                "stability_guard_output_topic",
                default_value="cmd_vel_stability",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    nav2_bringup_dir, "rviz", "nav2_default_view.rviz"
                ),
            ),
            DeclareLaunchArgument(
                "bt_nav_to_pose",
                default_value=os.path.join(
                    bringup_dir,
                    "behavior_trees",
                    "navigate_to_pose_no_spin.xml",
                ),
            ),
            DeclareLaunchArgument(
                "bt_nav_through_poses",
                default_value=os.path.join(
                    bringup_dir,
                    "behavior_trees",
                    "navigate_through_poses_no_spin.xml",
                ),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {"yaml_filename": map_file, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_amcl),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_broadcaster",
                output="screen",
                arguments=[
                    "--x",
                    map_to_odom_x,
                    "--y",
                    map_to_odom_y,
                    "--z",
                    map_to_odom_z,
                    "--roll",
                    map_to_odom_roll,
                    "--pitch",
                    map_to_odom_pitch,
                    "--yaw",
                    map_to_odom_yaw,
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_amcl,
                            "' == 'false' and '",
                            publish_map_to_odom_tf,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="keepout_filter_mask_server",
                output="screen",
                condition=IfCondition(use_costmap_filters),
                parameters=[
                    {
                        "yaml_filename": keepout_mask_file,
                        "topic_name": "keepout_filter_mask",
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="keepout_costmap_filter_info_server",
                output="screen",
                condition=IfCondition(use_costmap_filters),
                parameters=[
                    {
                        "type": 0,
                        "filter_info_topic": "/keepout_costmap_filter_info",
                        "mask_topic": "/keepout_filter_mask",
                        "base": 0.0,
                        "multiplier": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="speed_filter_mask_server",
                output="screen",
                condition=IfCondition(use_costmap_filters),
                parameters=[
                    {
                        "yaml_filename": speed_mask_file,
                        "topic_name": "speed_filter_mask",
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="speed_costmap_filter_info_server",
                output="screen",
                condition=IfCondition(use_costmap_filters),
                parameters=[
                    {
                        "type": 1,
                        "filter_info_topic": "/speed_costmap_filter_info",
                        "mask_topic": "/speed_filter_mask",
                        "base": 0.0,
                        "multiplier": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "default_nav_to_pose_bt_xml": bt_nav_to_pose,
                        "default_nav_through_poses_bt_xml": bt_nav_through_poses,
                    },
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("cmd_vel", velocity_smoother_input_topic),
                    (
                        "cmd_vel_smoothed",
                        PythonExpression(
                            [
                                "\"",
                                velocity_smoother_output_topic,
                                "\" if \"",
                                use_stability_guard,
                                "\" == \"true\" else (\"",
                                cmd_vel_in_topic,
                                "\" if \"",
                                use_collision_monitor,
                                "\" == \"true\" else \"",
                                velocity_smoother_passthrough_topic,
                                "\")",
                            ]
                        ),
                    ),
                ],
            ),
            Node(
                package="forklift_safety",
                executable="stability_guard",
                name="stability_guard",
                output="screen",
                condition=IfCondition(use_stability_guard),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": velocity_smoother_output_topic,
                        "output_topic": stability_guard_output_topic,
                        "profile_config_path": profile_config,
                        "load_profile": load_profile,
                        "lift_joint_name": "fork_base_joint",
                        "odom_topic": "/odom",
                        "imu_topic": "/imu",
                        "joint_state_topic": "/joint_states",
                    }
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                condition=IfCondition(use_collision_monitor),
                parameters=[
                    collision_monitor_file,
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_in_topic": cmd_vel_in_topic,
                        "cmd_vel_out_topic": cmd_vel_out_topic,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_safety",
                output="screen",
                condition=IfCondition(use_collision_monitor),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["collision_monitor"],
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                condition=IfCondition(use_amcl),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization_simple",
                output="screen",
                condition=UnlessCondition(use_amcl),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_filters",
                output="screen",
                condition=IfCondition(use_costmap_filters),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "keepout_filter_mask_server",
                            "keepout_costmap_filter_info_server",
                            "speed_filter_mask_server",
                            "speed_costmap_filter_info_server",
                        ],
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ],
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
