import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    simulation_dir = get_package_share_directory("forklift_simulation")
    description_dir = get_package_share_directory("forklift_description")
    control_dir = get_package_share_directory("forklift_control")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world = os.path.join(simulation_dir, "worlds", "small_warehouse_open_top.world")
    robot_xacro = os.path.join(description_dir, "urdf", "forklift_physics.xacro")
    rviz_config = os.path.join(description_dir, "rviz", "forklift_with_sensors.rviz")
    controller_config = os.path.join(control_dir, "config", "physics_controllers.yaml")
    actuator_config = os.path.join(control_dir, "config", "actuator_limits.yaml")

    robot_description = xacro.process_file(
        robot_xacro,
        mappings={
            "controller_config": controller_config,
            "steering_axle": "rear",
            "drive_axle": "front",
            "enable_depth_camera": "false",
        },
    ).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    effective_gui = PythonExpression(["'false' if '", headless, "' == 'true' else '", gui, "'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("force_software_rendering", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=world),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.30"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("publish_ground_truth", default_value="true"),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                [
                    "/usr/share/gazebo-11/models",
                    ":",
                    os.path.join(simulation_dir, "models"),
                    ":",
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GAZEBO_RESOURCE_PATH",
                ["/usr/share/gazebo-11:", EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                [
                    "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:",
                    EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable("OGRE_RESOURCE_PATH", "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                "1",
                condition=IfCondition(LaunchConfiguration("force_software_rendering")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": effective_gui,
                    "verbose": "true",
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "forklift_physics",
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
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["steering_position_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["drive_velocity_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="forklift_control",
                executable="ackermann_command_adapter",
                name="ackermann_command_adapter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "config_path": actuator_config}],
            ),
            Node(
                package="forklift_control",
                executable="wheel_odometry",
                name="wheel_odometry",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "odom_topic": "/wheel/odom", "publish_tf": False}],
            ),
            Node(
                package="forklift_control",
                executable="ground_truth_odom",
                name="ground_truth_odom",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_ground_truth")),
                parameters=[{"use_sim_time": use_sim_time, "entity_name": "forklift_physics"}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
