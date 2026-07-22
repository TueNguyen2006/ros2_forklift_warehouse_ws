import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from warehouse_visual_localization.launch_common import (
    build_visual_robot_description,
    get_common_paths,
    make_runtime_env_actions,
)


def generate_launch_description():
    paths = get_common_paths()
    visual_dir = paths["visual_dir"]
    navigation_dir = paths["navigation_dir"]
    simulation_dir = paths["simulation_dir"]
    gazebo_ros_dir = paths["gazebo_ros_dir"]
    forklift_robot_dir = paths["forklift_robot_dir"]
    ros_gazebo_plugin_dir = paths["ros_gazebo_plugin_dir"]

    default_world = os.path.join(simulation_dir, "worlds", "small_warehouse_open_top.world")
    default_visual_rviz = os.path.join(visual_dir, "config", "nav2_visualization.rviz")
    default_baseline_rviz = os.path.join(
        navigation_dir,
        "rviz",
        "forklift_nav_with_cameras.rviz",
    )
    planar_robot_description = build_visual_robot_description(simulation_dir, forklift_robot_dir)
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

    planar_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": planar_robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    planar_spawn_entity = Node(
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
            *make_runtime_env_actions(simulation_dir, ros_gazebo_plugin_dir),
            planar_robot_state_publisher,
            gazebo,
            planar_spawn_entity,
            rviz_node,
        ]
    )
