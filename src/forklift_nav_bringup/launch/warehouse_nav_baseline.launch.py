import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix, get_package_share_directory

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


GAZEBO_RESOURCE_DIR = "/usr/share/gazebo-11"
GAZEBO_SYSTEM_MODEL_DIR = "/usr/share/gazebo-11/models"
GAZEBO_SYSTEM_PLUGIN_DIR = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
OGRE_RESOURCE_DIR = "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"
BASE_COLLISION_LENGTH = 1.60
BASE_COLLISION_WIDTH = 1.10
BASE_COLLISION_HEIGHT = 1.30
BASE_COLLISION_X_OFFSET = 0.00
BASE_COLLISION_Z_OFFSET = 0.68
BASE_COLLISION_MASS = 1450.0
BASE_INERTIAL_Z_OFFSET = 0.34
NAV_BASE_OFFSET_X = 0.55
# GazeboRosPlanarMove applies angular velocity about the model center of mass,
# while Nav2 tracks base_footprint.  base_link is NAV_BASE_OFFSET_X behind
# base_footprint, so placing its inertial origin this far forward makes the
# model center of mass coincide with the navigation reference point.  If the
# inertial origin is left near base_link, every yaw correction also translates
# base_footprint and the path follower diverges from otherwise valid paths.
BASE_INERTIAL_X_OFFSET = NAV_BASE_OFFSET_X


def _set_or_create_origin(
    element: ElementTree.Element,
    xyz: str = "0 0 0",
    rpy: str = "0 0 0",
) -> None:
    origin = element.find("origin")
    if origin is None:
        origin = ElementTree.Element("origin")
        element.insert(0, origin)
    origin.set("xyz", xyz)
    origin.set("rpy", rpy)


def _normalize_wheel_kinematics(root: ElementTree.Element) -> None:
    wheel_joint_names = {"left_wheel_joint", "right_wheel_joint"}
    wheel_link_names = {"left_wheel", "right_wheel"}

    for joint in root.findall("joint"):
        if joint.attrib.get("name") not in wheel_joint_names:
            continue

        _set_or_create_origin(joint, xyz=joint.find("origin").attrib.get("xyz", "0 0 0"))
        axis = joint.find("axis")
        if axis is None:
            axis = ElementTree.SubElement(joint, "axis")
        axis.set("xyz", "0 1 0")

    for link in root.findall("link"):
        if link.attrib.get("name") not in wheel_link_names:
            continue

        for child_tag in ("visual", "collision", "inertial"):
            child = link.find(child_tag)
            if child is not None:
                _set_or_create_origin(child, rpy="-1.57079632679 0 0")


def _strip_drive_joints_from_ros2_control(root: ElementTree.Element) -> None:
    ros2_control = root.find("ros2_control")
    if ros2_control is None:
        return

    for joint in list(ros2_control.findall("joint")):
        if joint.attrib.get("name") in {"left_wheel_joint", "right_wheel_joint"}:
            ros2_control.remove(joint)


def _remove_link_dynamics(root: ElementTree.Element) -> None:
    for link in root.findall("link"):
        if link.attrib.get("name") == "base_link":
            continue

        for child_tag in ("collision", "inertial"):
            for child in list(link.findall(child_tag)):
                link.remove(child)


def _ensure_base_footprint_root(root: ElementTree.Element) -> None:
    if root.find("./link[@name='base_footprint']") is not None:
        return

    base_footprint = ElementTree.Element("link", {"name": "base_footprint"})
    base_footprint_joint = ElementTree.Element(
        "joint",
        {"name": "base_footprint_joint", "type": "fixed"},
    )
    ElementTree.SubElement(base_footprint_joint, "parent", {"link": "base_footprint"})
    ElementTree.SubElement(base_footprint_joint, "child", {"link": "base_link"})
    ElementTree.SubElement(
        base_footprint_joint,
        "origin",
        {"xyz": f"-{NAV_BASE_OFFSET_X} 0 0", "rpy": "0 0 0"},
    )

    root.insert(0, base_footprint)
    root.insert(1, base_footprint_joint)


def _make_simple_rigid_joint_tree(root: ElementTree.Element) -> None:
    for joint in root.findall("joint"):
        if joint.attrib.get("name") in {
            "left_wheel_joint",
            "right_wheel_joint",
            "fork_base_joint",
        }:
            joint.set("type", "fixed")
            axis = joint.find("axis")
            if axis is not None:
                joint.remove(axis)
            limit = joint.find("limit")
            if limit is not None:
                joint.remove(limit)


def _configure_simple_collision_body(root: ElementTree.Element) -> None:
    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        return

    for child_tag in ("collision", "inertial"):
        for child in list(base_link.findall(child_tag)):
            base_link.remove(child)

    collision = ElementTree.SubElement(
        base_link,
        "collision",
        {"name": "base_link_collision"},
    )
    _set_or_create_origin(
        collision,
        xyz=f"{BASE_COLLISION_X_OFFSET} 0 {BASE_COLLISION_Z_OFFSET}",
    )
    collision_geometry = ElementTree.SubElement(collision, "geometry")
    ElementTree.SubElement(
        collision_geometry,
        "box",
        {
            "size": (
                f"{BASE_COLLISION_LENGTH} "
                f"{BASE_COLLISION_WIDTH} "
                f"{BASE_COLLISION_HEIGHT}"
            )
        },
    )

    ixx = (BASE_COLLISION_MASS / 12.0) * (
        (BASE_COLLISION_WIDTH**2) + (BASE_COLLISION_HEIGHT**2)
    )
    iyy = (BASE_COLLISION_MASS / 12.0) * (
        (BASE_COLLISION_LENGTH**2) + (BASE_COLLISION_HEIGHT**2)
    )
    izz = (BASE_COLLISION_MASS / 12.0) * (
        (BASE_COLLISION_LENGTH**2) + (BASE_COLLISION_WIDTH**2)
    )

    inertial = ElementTree.SubElement(base_link, "inertial")
    _set_or_create_origin(
        inertial,
        xyz=f"{BASE_INERTIAL_X_OFFSET} 0 {BASE_INERTIAL_Z_OFFSET}",
    )
    ElementTree.SubElement(inertial, "mass", {"value": str(BASE_COLLISION_MASS)})
    ElementTree.SubElement(
        inertial,
        "inertia",
        {
            "ixx": str(ixx),
            "ixy": "0.0",
            "ixz": "0.0",
            "iyy": str(iyy),
            "iyz": "0.0",
            "izz": str(izz),
        },
    )


def _configure_planar_base(root: ElementTree.Element) -> None:
    ros2_control = root.find("ros2_control")
    if ros2_control is not None:
        root.remove(ros2_control)

    for gazebo_element in list(root.findall("gazebo")):
        plugin_filenames = {
            plugin.attrib.get("filename", "")
            for plugin in gazebo_element.iter("plugin")
        }
        if "libgazebo_ros_diff_drive.so" in plugin_filenames:
            root.remove(gazebo_element)
            continue
        if "libgazebo_ros2_control.so" in plugin_filenames:
            root.remove(gazebo_element)

    # This profile is deliberately kinematic.  Gravity/contact resolution can
    # otherwise lift and pitch the model while GazeboRosPlanarMove continues to
    # publish a planar odom frame.  Once roll/pitch grows, the projected yaw can
    # jump by about 90 degrees and both Smac replanning and RPP path tracking
    # become unstable.  Horizontal obstacle avoidance remains Nav2's job.
    base_link_gazebo = ElementTree.SubElement(
        root,
        "gazebo",
        {"reference": "base_link"},
    )
    ElementTree.SubElement(base_link_gazebo, "gravity").text = "false"

    planar_move = ElementTree.SubElement(root, "gazebo")
    plugin = ElementTree.SubElement(
        planar_move,
        "plugin",
        {"name": "planar_move", "filename": "libgazebo_ros_planar_move.so"},
    )
    ros = ElementTree.SubElement(plugin, "ros")
    ElementTree.SubElement(ros, "remapping").text = "cmd_vel:=/cmd_vel"
    ElementTree.SubElement(ros, "remapping").text = "odom:=/odom"
    ElementTree.SubElement(plugin, "update_rate").text = "100.0"
    ElementTree.SubElement(plugin, "publish_rate").text = "30.0"
    ElementTree.SubElement(plugin, "publish_odom").text = "true"
    ElementTree.SubElement(plugin, "publish_odom_tf").text = "true"
    ElementTree.SubElement(plugin, "odometry_frame").text = "odom"
    ElementTree.SubElement(plugin, "robot_base_frame").text = "base_footprint"
    ElementTree.SubElement(plugin, "covariance_x").text = "0.0001"
    ElementTree.SubElement(plugin, "covariance_y").text = "0.0001"
    ElementTree.SubElement(plugin, "covariance_yaw").text = "0.01"


def _disable_lidar_visualization(root: ElementTree.Element) -> None:
    for gazebo_element in root.findall("gazebo"):
        if gazebo_element.attrib.get("reference") != "laser_frame_link":
            continue

        for sensor in gazebo_element.findall("sensor"):
            if sensor.attrib.get("type") != "ray":
                continue

            visualize = sensor.find("visualize")
            if visualize is None:
                visualize = ElementTree.SubElement(sensor, "visualize")
            visualize.text = "false"


def _remove_existing_camera_assets(root: ElementTree.Element) -> None:
    for gazebo_element in list(root.findall("gazebo")):
        if gazebo_element.attrib.get("reference") == "camera_link":
            root.remove(gazebo_element)

    for joint in list(root.findall("joint")):
        if joint.attrib.get("name") in {"camera_joint", "camera_optical_joint"}:
            root.remove(joint)

    for link in list(root.findall("link")):
        if link.attrib.get("name") in {"camera_link", "camera_link_optical"}:
            root.remove(link)


def _add_top_camera_suite(root: ElementTree.Element) -> None:
    if root.find("./link[@name='rgb_camera_link']") is not None:
        return

    def add_link(name: str) -> None:
        root.append(ElementTree.Element("link", {"name": name}))

    def add_fixed_joint(
        name: str,
        parent: str,
        child: str,
        xyz: str,
        rpy: str,
    ) -> None:
        joint = ElementTree.Element("joint", {"name": name, "type": "fixed"})
        ElementTree.SubElement(joint, "parent", {"link": parent})
        ElementTree.SubElement(joint, "child", {"link": child})
        ElementTree.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
        root.append(joint)

    add_link("rgb_camera_link")
    add_link("rgb_camera_optical_link")
    add_link("depth_camera_link")
    add_link("depth_camera_optical_link")

    add_fixed_joint(
        "rgb_camera_joint",
        "chassis_top_link",
        "rgb_camera_link",
        "0.58 0.0 1.52",
        "0 0 1.57079632679",
    )
    add_fixed_joint(
        "rgb_camera_optical_joint",
        "rgb_camera_link",
        "rgb_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )
    add_fixed_joint(
        "depth_camera_joint",
        "chassis_top_link",
        "depth_camera_link",
        "0.52 0.0 1.42",
        "0 0 1.57079632679",
    )
    add_fixed_joint(
        "depth_camera_optical_joint",
        "depth_camera_link",
        "depth_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )

    rgb_gazebo = ElementTree.Element("gazebo", {"reference": "rgb_camera_link"})
    rgb_sensor = ElementTree.SubElement(
        rgb_gazebo,
        "sensor",
        {"name": "rgb_camera_sensor", "type": "camera"},
    )
    ElementTree.SubElement(rgb_sensor, "pose").text = "0 0 0 0 0 0"
    ElementTree.SubElement(rgb_sensor, "visualize").text = "false"
    ElementTree.SubElement(rgb_sensor, "update_rate").text = "15"
    rgb_camera = ElementTree.SubElement(rgb_sensor, "camera")
    ElementTree.SubElement(rgb_camera, "horizontal_fov").text = "1.089"
    rgb_image = ElementTree.SubElement(rgb_camera, "image")
    ElementTree.SubElement(rgb_image, "format").text = "R8G8B8"
    ElementTree.SubElement(rgb_image, "width").text = "640"
    ElementTree.SubElement(rgb_image, "height").text = "480"
    rgb_clip = ElementTree.SubElement(rgb_camera, "clip")
    ElementTree.SubElement(rgb_clip, "near").text = "0.05"
    ElementTree.SubElement(rgb_clip, "far").text = "12.0"
    rgb_plugin = ElementTree.SubElement(
        rgb_sensor,
        "plugin",
        {
            "name": "rgb_camera_controller",
            "filename": "libgazebo_ros_camera.so",
        },
    )
    ElementTree.SubElement(rgb_plugin, "frame_name").text = "rgb_camera_optical_link"
    ElementTree.SubElement(rgb_plugin, "camera_name").text = "rgb_camera"
    root.append(rgb_gazebo)

    depth_gazebo = ElementTree.Element("gazebo", {"reference": "depth_camera_link"})
    depth_sensor = ElementTree.SubElement(
        depth_gazebo,
        "sensor",
        {"name": "depth_camera_sensor", "type": "depth"},
    )
    ElementTree.SubElement(depth_sensor, "pose").text = "0 0 0 0 0 0"
    ElementTree.SubElement(depth_sensor, "visualize").text = "false"
    ElementTree.SubElement(depth_sensor, "update_rate").text = "10"
    depth_camera = ElementTree.SubElement(depth_sensor, "camera")
    ElementTree.SubElement(depth_camera, "horizontal_fov").text = "1.089"
    depth_image = ElementTree.SubElement(depth_camera, "image")
    ElementTree.SubElement(depth_image, "format").text = "B8G8R8"
    ElementTree.SubElement(depth_image, "width").text = "640"
    ElementTree.SubElement(depth_image, "height").text = "480"
    depth_clip = ElementTree.SubElement(depth_camera, "clip")
    ElementTree.SubElement(depth_clip, "near").text = "0.05"
    ElementTree.SubElement(depth_clip, "far").text = "12.0"
    depth_plugin = ElementTree.SubElement(
        depth_sensor,
        "plugin",
        {
            "name": "depth_camera_controller",
            "filename": "libgazebo_ros_camera.so",
        },
    )
    ElementTree.SubElement(depth_plugin, "frame_name").text = (
        "depth_camera_optical_link"
    )
    ElementTree.SubElement(depth_plugin, "camera_name").text = "depth_camera"
    ElementTree.SubElement(depth_plugin, "min_depth").text = "0.10"
    ElementTree.SubElement(depth_plugin, "max_depth").text = "12.0"
    root.append(depth_gazebo)


def _build_baseline_robot_description(forklift_robot_dir: str) -> str:
    robot_description = xacro.process_file(
        os.path.join(forklift_robot_dir, "forklift.urdf.xacro")
    ).toxml()
    robot_description = robot_description.replace(
        'length="laser_frame_length"',
        'length="0.04"',
    )

    root = ElementTree.fromstring(robot_description)
    for gazebo_element in list(root.findall("gazebo")):
        reference = gazebo_element.attrib.get("reference", "")
        plugin_filenames = {
            plugin.attrib.get("filename", "")
            for plugin in gazebo_element.iter("plugin")
        }
        if "libros_collision_detection.so" in plugin_filenames:
            root.remove(gazebo_element)
            continue
        if reference == "camera_link" and "libgazebo_ros_camera.so" in plugin_filenames:
            root.remove(gazebo_element)
            continue

    _ensure_base_footprint_root(root)
    _disable_lidar_visualization(root)
    _remove_existing_camera_assets(root)
    _add_top_camera_suite(root)
    _remove_link_dynamics(root)
    _make_simple_rigid_joint_tree(root)
    _configure_simple_collision_body(root)
    _configure_planar_base(root)

    return ElementTree.tostring(root, encoding="unicode")


def generate_launch_description():
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    forklift_robot_dir = get_package_share_directory("forklift_robot")
    ros_gazebo_plugins_prefix = get_package_prefix("gazebo_plugins")
    ros_gazebo_plugin_dir = os.path.join(ros_gazebo_plugins_prefix, "lib")

    default_world = os.path.join(
        bringup_dir, "worlds", "small_warehouse_open_top.world"
    )
    default_map = os.path.join(bringup_dir, "maps", "warehouse_map.yaml")
    default_nav_params = os.path.join(bringup_dir, "config", "nav2_params_simple.yaml")
    default_collision_params = os.path.join(
        bringup_dir, "config", "collision_monitor_smoke.yaml"
    )

    robot_description = _build_baseline_robot_description(forklift_robot_dir)

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    load_profile = LaunchConfiguration("load_profile")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    velocity_smoother_output_topic = LaunchConfiguration(
        "velocity_smoother_output_topic"
    )
    stability_guard_output_topic = LaunchConfiguration(
        "stability_guard_output_topic"
    )
    use_amcl = LaunchConfiguration("use_amcl")
    use_costmap_filters = LaunchConfiguration("use_costmap_filters")
    use_stability_guard = LaunchConfiguration("use_stability_guard")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    world_file = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    collision_monitor_file = LaunchConfiguration("collision_monitor_file")
    keepout_mask = LaunchConfiguration("keepout_mask")
    speed_mask = LaunchConfiguration("speed_mask")
    use_initial_pose_publisher = LaunchConfiguration("use_initial_pose_publisher")
    enable_debug_logger = LaunchConfiguration("enable_debug_logger")
    debug_log_dir = LaunchConfiguration("debug_log_dir")
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
            "forklift_baseline",
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
        condition=IfCondition(use_initial_pose_publisher),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "initial_x": spawn_x,
                "initial_y": spawn_y,
                "initial_yaw": spawn_yaw,
            }
        ],
    )

    nav_debug_logger = Node(
        package="forklift_nav_bringup",
        executable="nav_debug_logger",
        name="nav_debug_logger",
        output="screen",
        condition=IfCondition(enable_debug_logger),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "log_dir": debug_log_dir,
                "global_frame": "map",
                "odom_frame": "odom",
                "robot_frame": "base_footprint",
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
            "params_file": params_file,
            "collision_monitor_file": collision_monitor_file,
            "map": map_file,
            "keepout_mask": keepout_mask,
            "speed_mask": speed_mask,
            "load_profile": load_profile,
            "cmd_vel_in_topic": cmd_vel_in_topic,
            "cmd_vel_out_topic": cmd_vel_out_topic,
            "velocity_smoother_output_topic": velocity_smoother_output_topic,
            "stability_guard_output_topic": stability_guard_output_topic,
            "use_amcl": use_amcl,
            "use_costmap_filters": use_costmap_filters,
            "use_stability_guard": use_stability_guard,
            "use_collision_monitor": use_collision_monitor,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("use_amcl", default_value="false"),
            DeclareLaunchArgument("use_costmap_filters", default_value="false"),
            DeclareLaunchArgument("use_stability_guard", default_value="false"),
            DeclareLaunchArgument("use_collision_monitor", default_value="false"),
            DeclareLaunchArgument("cmd_vel_in_topic", default_value="cmd_vel_smoothed"),
            DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "velocity_smoother_output_topic",
                default_value="/cmd_vel",
            ),
            DeclareLaunchArgument(
                "stability_guard_output_topic",
                default_value="/cmd_vel",
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_initial_pose_publisher", default_value="false"),
            DeclareLaunchArgument("enable_debug_logger", default_value="true"),
            DeclareLaunchArgument(
                "debug_log_dir",
                default_value=os.path.join(
                    os.path.expanduser("~"),
                    ".ros",
                    "forklift_nav_logs",
                ),
            ),
            DeclareLaunchArgument("params_file", default_value=default_nav_params),
            DeclareLaunchArgument(
                "collision_monitor_file",
                default_value=default_collision_params,
            ),
            DeclareLaunchArgument(
                "keepout_mask",
                default_value=os.path.join(
                    bringup_dir, "maps", "warehouse_keepout_mask.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "speed_mask",
                default_value=os.path.join(
                    bringup_dir, "maps", "warehouse_speed_mask.yaml"
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
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
                        odom_to_imu,
                        initial_pose_publisher,
                        nav_stack,
                        nav_debug_logger,
                    ],
                )
            ),
        ]
    )
