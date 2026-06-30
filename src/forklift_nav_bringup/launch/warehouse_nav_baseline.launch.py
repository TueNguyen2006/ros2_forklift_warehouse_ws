import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from forklift_nav_bringup.world_map_generator import default_output_root, ensure_world_map
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
# Gazebo Classic on WSLg struggles when four camera sensors run at high rate.
# Prioritize the RGB-D depth stream used by RTAB-Map and keep the standalone
# RGB / stereo sensors as lower-rate debug feeds.
RGB_CAMERA_UPDATE_RATE = 6
RGB_CAMERA_WIDTH = 320
RGB_CAMERA_HEIGHT = 240
DEPTH_CAMERA_WIDTH = 424
DEPTH_CAMERA_HEIGHT = 240
DEPTH_CAMERA_STABLE_UPDATE_RATE = 10
STEREO_CAMERA_UPDATE_RATE = 6
STEREO_CAMERA_WIDTH = 320
STEREO_CAMERA_HEIGHT = 240
STEREO_BASELINE = 0.12
# Mount the simulated cameras fully outside the forklift body so the RGB-D and
# stereo views are not occluded by the red chassis / mast geometry.
CAMERA_MOUNT_X = 2.12
CAMERA_MOUNT_Z = 2.28
# GazeboRosPlanarMove applies angular velocity about the model center of mass,
# while Nav2 tracks base_footprint.  base_link is NAV_BASE_OFFSET_X behind
# base_footprint, so placing its inertial origin this far forward makes the
# model center of mass coincide with the navigation reference point.  If the
# inertial origin is left near base_link, every yaw correction also translates
# base_footprint and the path follower diverges from otherwise valid paths.
BASE_INERTIAL_X_OFFSET = NAV_BASE_OFFSET_X


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _configure_runtime_map_assets(context, *_, **__):
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
                "Using generated map assets: "
                f"map={artifacts.map_yaml} "
                f"keepout={artifacts.keepout_yaml} "
                f"speed={artifacts.speed_yaml}"
            )
        ),
    ]


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
    camera_link_names = {
        "camera_link",
        "camera_link_optical",
        "rgb_camera_link",
        "rgb_camera_optical_link",
        "depth_camera_link",
        "depth_camera_optical_link",
        "stereo_left_camera_link",
        "stereo_left_camera_optical_link",
        "stereo_right_camera_link",
        "stereo_right_camera_optical_link",
    }
    camera_joint_names = {
        "camera_joint",
        "camera_optical_joint",
        "rgb_camera_joint",
        "rgb_camera_optical_joint",
        "depth_camera_joint",
        "depth_camera_optical_joint",
        "stereo_left_camera_joint",
        "stereo_left_camera_optical_joint",
        "stereo_right_camera_joint",
        "stereo_right_camera_optical_joint",
    }
    for gazebo_element in list(root.findall("gazebo")):
        if gazebo_element.attrib.get("reference") in camera_link_names:
            root.remove(gazebo_element)

    for joint in list(root.findall("joint")):
        if joint.attrib.get("name") in camera_joint_names:
            root.remove(joint)

    for link in list(root.findall("link")):
        if link.attrib.get("name") in camera_link_names:
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

    def add_camera_sensor(
        *,
        link_name: str,
        sensor_name: str,
        sensor_type: str,
        plugin_name: str,
        camera_name: str,
        frame_name: str,
        width: int,
        height: int,
        update_rate: int,
        image_format: str,
        near_clip: str = "0.05",
        far_clip: str = "12.0",
        min_depth: str | None = None,
        max_depth: str | None = None,
    ) -> None:
        gazebo = ElementTree.Element("gazebo", {"reference": link_name})
        sensor = ElementTree.SubElement(
            gazebo,
            "sensor",
            {"name": sensor_name, "type": sensor_type},
        )
        ElementTree.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
        ElementTree.SubElement(sensor, "always_on").text = "true"
        ElementTree.SubElement(sensor, "visualize").text = "false"
        ElementTree.SubElement(sensor, "update_rate").text = str(update_rate)
        camera = ElementTree.SubElement(sensor, "camera")
        ElementTree.SubElement(camera, "horizontal_fov").text = "1.089"
        image = ElementTree.SubElement(camera, "image")
        ElementTree.SubElement(image, "format").text = image_format
        ElementTree.SubElement(image, "width").text = str(width)
        ElementTree.SubElement(image, "height").text = str(height)
        clip = ElementTree.SubElement(camera, "clip")
        ElementTree.SubElement(clip, "near").text = near_clip
        ElementTree.SubElement(clip, "far").text = far_clip
        plugin = ElementTree.SubElement(
            sensor,
            "plugin",
            {"name": plugin_name, "filename": "libgazebo_ros_camera.so"},
        )
        ElementTree.SubElement(plugin, "frame_name").text = frame_name
        ElementTree.SubElement(plugin, "camera_name").text = camera_name
        if min_depth is not None:
            ElementTree.SubElement(plugin, "min_depth").text = min_depth
        if max_depth is not None:
            ElementTree.SubElement(plugin, "max_depth").text = max_depth
        root.append(gazebo)

    add_link("rgb_camera_link")
    add_link("rgb_camera_optical_link")
    add_link("depth_camera_link")
    add_link("depth_camera_optical_link")
    add_link("stereo_left_camera_link")
    add_link("stereo_left_camera_optical_link")
    add_link("stereo_right_camera_link")
    add_link("stereo_right_camera_optical_link")

    add_fixed_joint(
        "rgb_camera_joint",
        "chassis_top_link",
        "rgb_camera_link",
        f"{CAMERA_MOUNT_X} 0.0 {CAMERA_MOUNT_Z}",
        "0 0 0",
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
        f"{CAMERA_MOUNT_X + 0.03} 0.0 {CAMERA_MOUNT_Z - 0.08}",
        "0 0 0",
    )
    add_fixed_joint(
        "depth_camera_optical_joint",
        "depth_camera_link",
        "depth_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )
    add_fixed_joint(
        "stereo_left_camera_joint",
        "chassis_top_link",
        "stereo_left_camera_link",
        f"{CAMERA_MOUNT_X - 0.02} {STEREO_BASELINE / 2.0} {CAMERA_MOUNT_Z - 0.02}",
        "0 0 0",
    )
    add_fixed_joint(
        "stereo_left_camera_optical_joint",
        "stereo_left_camera_link",
        "stereo_left_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )
    add_fixed_joint(
        "stereo_right_camera_joint",
        "chassis_top_link",
        "stereo_right_camera_link",
        f"{CAMERA_MOUNT_X - 0.02} -{STEREO_BASELINE / 2.0} {CAMERA_MOUNT_Z - 0.02}",
        "0 0 0",
    )
    add_fixed_joint(
        "stereo_right_camera_optical_joint",
        "stereo_right_camera_link",
        "stereo_right_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )

    add_camera_sensor(
        link_name="rgb_camera_link",
        sensor_name="rgb_camera_sensor",
        sensor_type="camera",
        plugin_name="rgb_camera_controller",
        camera_name="rgb_camera",
        frame_name="rgb_camera_optical_link",
        width=RGB_CAMERA_WIDTH,
        height=RGB_CAMERA_HEIGHT,
        update_rate=RGB_CAMERA_UPDATE_RATE,
        image_format="R8G8B8",
    )
    add_camera_sensor(
        link_name="depth_camera_link",
        sensor_name="depth_camera_sensor",
        sensor_type="depth",
        plugin_name="depth_camera_controller",
        camera_name="depth_camera",
        frame_name="depth_camera_optical_link",
        width=DEPTH_CAMERA_WIDTH,
        height=DEPTH_CAMERA_HEIGHT,
        update_rate=DEPTH_CAMERA_STABLE_UPDATE_RATE,
        image_format="R8G8B8",
        min_depth="0.10",
        max_depth="12.0",
    )
    add_camera_sensor(
        link_name="stereo_left_camera_link",
        sensor_name="stereo_left_camera_sensor",
        sensor_type="camera",
        plugin_name="stereo_left_camera_controller",
        camera_name="stereo_left_camera",
        frame_name="stereo_left_camera_optical_link",
        width=STEREO_CAMERA_WIDTH,
        height=STEREO_CAMERA_HEIGHT,
        update_rate=STEREO_CAMERA_UPDATE_RATE,
        image_format="R8G8B8",
    )
    add_camera_sensor(
        link_name="stereo_right_camera_link",
        sensor_name="stereo_right_camera_sensor",
        sensor_type="camera",
        plugin_name="stereo_right_camera_controller",
        camera_name="stereo_right_camera",
        frame_name="stereo_right_camera_optical_link",
        width=STEREO_CAMERA_WIDTH,
        height=STEREO_CAMERA_HEIGHT,
        update_rate=STEREO_CAMERA_UPDATE_RATE,
        image_format="R8G8B8",
    )


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
    gazebo_goal_tool_prefix = get_package_prefix("gazebo_nav_goal_tool")
    gazebo_goal_tool_plugin_dir = os.path.join(gazebo_goal_tool_prefix, "lib")
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
    default_rviz_config = os.path.join(
        bringup_dir, "rviz", "forklift_nav_with_cameras.rviz"
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
    mesa_adapter_name = LaunchConfiguration("mesa_adapter_name")
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
    rviz_config = LaunchConfiguration("rviz_config")
    use_initial_pose_publisher = LaunchConfiguration("use_initial_pose_publisher")
    enable_debug_logger = LaunchConfiguration("enable_debug_logger")
    debug_log_dir = LaunchConfiguration("debug_log_dir")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    libgl_software = PythonExpression(["'1' if '", gui, "' == 'false' else '0'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "true",
            "extra_gazebo_args": "-slibgazebo_ros_state.so",
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
            "map": LaunchConfiguration("resolved_map"),
            "keepout_mask": LaunchConfiguration("resolved_keepout_mask"),
            "speed_mask": LaunchConfiguration("resolved_speed_mask"),
            "rviz_config": rviz_config,
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
            DeclareLaunchArgument("auto_generate_map", default_value="true"),
            DeclareLaunchArgument(
                "generated_map_root",
                default_value=str(default_output_root()),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("mesa_adapter_name", default_value="NVIDIA"),
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
                "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                mesa_adapter_name,
            ),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                libgl_software,
            ),
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
                    gazebo_goal_tool_plugin_dir,
                    ":",
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
                    gazebo_goal_tool_plugin_dir,
                    ":",
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
            OpaqueFunction(function=_configure_runtime_map_assets),
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
