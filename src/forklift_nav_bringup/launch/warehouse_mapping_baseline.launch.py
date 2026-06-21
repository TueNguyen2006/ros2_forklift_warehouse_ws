import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    forklift_robot_dir = get_package_share_directory("forklift_robot")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    world_file = os.path.join(bringup_dir, "worlds", "small_warehouse.world")
    rviz_config = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    robot_description = xacro.process_file(
        os.path.join(forklift_robot_dir, "forklift.urdf.xacro")
    ).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_file, "gui": gui, "verbose": "true"}.items(),
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
            "forklift_mapping",
            "-topic",
            "robot_description",
            "-x",
            "-3.071",
            "-y",
            "3.583",
            "-z",
            "0.05",
            "-Y",
            "-1.57",
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    fork_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fork_joint_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    odom_to_imu = Node(
        package="forklift_safety",
        executable="odom_to_imu",
        name="odom_to_imu",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_odom_topic": "/odom",
                "output_imu_topic": "/imu",
            }
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "mapping",
                "resolution": 0.05,
                "minimum_time_interval": 0.5,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                [
                    os.path.join(bringup_dir, "models"),
                    ":",
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            robot_state_publisher,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster, fork_joint_controller],
                )
            ),
            odom_to_imu,
            slam_toolbox,
            rviz_node,
        ]
    )
