import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro


GAZEBO_RESOURCE_DIR = "/usr/share/gazebo-11"
GAZEBO_SYSTEM_MODEL_DIR = "/usr/share/gazebo-11/models"
GAZEBO_SYSTEM_PLUGIN_DIR = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
OGRE_RESOURCE_DIR = "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    description_dir = get_package_share_directory("forklift_description_realistic")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    ros_gazebo_plugins_prefix = get_package_prefix("gazebo_plugins")
    ros_gazebo_plugin_dir = os.path.join(ros_gazebo_plugins_prefix, "lib")
    world_file = os.path.join(bringup_dir, "worlds", "small_warehouse.world")
    nav_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    collision_params = os.path.join(
        bringup_dir, "config", "collision_monitor.yaml"
    )

    robot_description = xacro.process_file(
        os.path.join(description_dir, "urdf", "rear_steer_forklift.urdf.xacro")
    ).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    load_profile = LaunchConfiguration("load_profile")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

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
            "forklift_rear_steer",
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
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
        ],
        output="screen",
    )

    rear_steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_steer_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    lift_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lift_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    controller_odom_bridge = Node(
        package="forklift_nav_bringup",
        executable="controller_odom_bridge",
        name="controller_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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

    initial_pose_publisher = Node(
        package="forklift_nav_bringup",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "initial_x": spawn_x,
                "initial_y": spawn_y,
                "initial_yaw": spawn_yaw,
            }
        ],
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "forklift_nav_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": rviz,
            "params_file": nav_params,
            "collision_monitor_file": collision_params,
            "load_profile": load_profile,
            "cmd_vel_out_topic": "/rear_steer_controller/reference_unstamped",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("spawn_x", default_value="-4.8"),
            DeclareLaunchArgument("spawn_y", default_value="-8.2"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                [
                    os.path.join(bringup_dir, "models"),
                    ":",
                    GAZEBO_SYSTEM_MODEL_DIR,
                    ":",
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                [
                    ros_gazebo_plugin_dir,
                    ":",
                    GAZEBO_SYSTEM_PLUGIN_DIR,
                    ":",
                    EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GAZEBO_RESOURCE_PATH",
                [
                    GAZEBO_RESOURCE_DIR,
                    ":",
                    EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    ros_gazebo_plugin_dir,
                    ":",
                    GAZEBO_SYSTEM_PLUGIN_DIR,
                    ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "OGRE_RESOURCE_PATH",
                [
                    OGRE_RESOURCE_DIR,
                    ":",
                    EnvironmentVariable("OGRE_RESOURCE_PATH", default_value=""),
                ],
            ),
            robot_state_publisher,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        joint_state_broadcaster,
                        rear_steer_controller,
                        lift_position_controller,
                    ],
                )
            ),
            controller_odom_bridge,
            odom_to_imu,
            initial_pose_publisher,
            nav_stack,
        ]
    )
