import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def _remove_gazebo_plugins(
    root: ElementTree.Element,
    plugin_filenames: set[str],
) -> None:
    for gazebo_element in list(root.findall("gazebo")):
        found_plugins = {
            plugin.attrib.get("filename", "")
            for plugin in gazebo_element.iter("plugin")
        }
        if found_plugins.intersection(plugin_filenames):
            root.remove(gazebo_element)


def _build_cangozpi_robot_description(forklift_robot_dir: str) -> str:
    robot_description = xacro.process_file(
        os.path.join(forklift_robot_dir, "forklift.urdf.xacro")
    ).toxml()

    # Upstream has this literal in lidar.xacro; URDF parsers expect a number.
    robot_description = robot_description.replace(
        'length="laser_frame_length"',
        'length="0.04"',
    )

    root = ElementTree.fromstring(robot_description)

    ros2_control = root.find("ros2_control")
    if ros2_control is not None:
        root.remove(ros2_control)

    _remove_gazebo_plugins(
        root,
        {
            "libgazebo_ros2_control.so",
            "libros_collision_detection.so",
        },
    )

    return ElementTree.tostring(root, encoding="unicode")


def generate_launch_description():
    simulation_dir = get_package_share_directory("forklift_simulation")
    forklift_robot_dir = get_package_share_directory("forklift_robot")
    ros_gazebo_plugins_prefix = get_package_prefix("ros_gazebo_plugins")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    default_world = os.path.join(simulation_dir, "worlds", "physics_floor.world")
    rviz_config = os.path.join(forklift_robot_dir, "forklift_with_sensors.rviz")
    robot_description = _build_cangozpi_robot_description(forklift_robot_dir)

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    effective_gui = PythonExpression(
        ["'false' if '", headless, "' == 'true' else '", gui, "'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("force_software_rendering", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
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
                "GAZEBO_PLUGIN_PATH",
                [
                    os.path.join(
                        ros_gazebo_plugins_prefix,
                        "lib",
                        "ros_gazebo_plugins",
                    ),
                    ":",
                    "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:",
                    EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GAZEBO_RESOURCE_PATH",
                [
                    "/usr/share/gazebo-11:",
                    EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
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
                    "forklift_bot",
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
