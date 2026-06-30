import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from warehouse_visual_localization.launch_common import (
    build_visual_rear_steer_robot_description,
    build_visual_robot_description,
    get_common_paths,
    make_runtime_env_actions,
)


def generate_launch_description():
    paths = get_common_paths()
    visual_dir = paths["visual_dir"]
    bringup_dir = paths["bringup_dir"]
    gazebo_ros_dir = paths["gazebo_ros_dir"]
    forklift_robot_dir = paths["forklift_robot_dir"]
    realistic_dir = paths["realistic_dir"]
    ros_gazebo_plugin_dir = paths["ros_gazebo_plugin_dir"]

    default_world = os.path.join(bringup_dir, "worlds", "small_warehouse_open_top.world")
    default_visual_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    default_baseline_rviz = os.path.join(
        bringup_dir,
        "rviz",
        "forklift_nav_with_cameras.rviz",
    )
    rear_steer_controller_config = os.path.join(
        visual_dir, "config", "rear_steer_controller_visual.yaml"
    )

    planar_robot_description = build_visual_robot_description(bringup_dir, forklift_robot_dir)
    rear_steer_robot_description = build_visual_rear_steer_robot_description(
        realistic_dir, rear_steer_controller_config
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")
    drive_model = LaunchConfiguration("drive_model")
    effective_gui = PythonExpression(
        ["'false' if '", headless, "' == 'true' else '", gui, "'"]
    )
    use_planar_drive = IfCondition(PythonExpression(["'", drive_model, "' == 'planar'"]))
    use_rear_steer_drive = IfCondition(
        PythonExpression(["'", drive_model, "' == 'rear_steer'"])
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

    planar_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=use_planar_drive,
        parameters=[
            {
                "robot_description": planar_robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rear_steer_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=use_rear_steer_drive,
        parameters=[
            {
                "robot_description": rear_steer_robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    planar_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        condition=use_planar_drive,
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

    rear_steer_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        condition=use_rear_steer_drive,
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

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=use_rear_steer_drive,
    )

    rear_steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rear_steer_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=use_rear_steer_drive,
    )

    lift_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=use_rear_steer_drive,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "rviz",
                default_value=PythonExpression(
                    ["'false' if '", headless, "' == 'true' else 'true'"]
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("rviz_mode", default_value="visual"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PythonExpression(
                    [
                        "'",
                        default_baseline_rviz,
                        "' if '",
                        LaunchConfiguration("rviz_mode"),
                        "' == 'baseline' else '",
                        default_visual_rviz,
                        "'",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_prefix",
                default_value=os.path.expanduser("~/ros2_local_overlay/opt/ros/humble"),
            ),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
            DeclareLaunchArgument(
                "force_software_rendering",
                default_value=EnvironmentVariable(
                    "VISUAL_LOCALIZATION_FORCE_SOFTWARE_RENDERING",
                    default_value="false",
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("drive_model", default_value="planar"),
            *make_runtime_env_actions(bringup_dir, ros_gazebo_plugin_dir),
            planar_robot_state_publisher,
            rear_steer_robot_state_publisher,
            gazebo,
            planar_spawn_entity,
            rear_steer_spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rear_steer_spawn_entity,
                    on_exit=[
                        joint_state_broadcaster,
                        rear_steer_controller,
                        lift_position_controller,
                    ],
                )
            ),
            rviz_node,
        ]
    )
