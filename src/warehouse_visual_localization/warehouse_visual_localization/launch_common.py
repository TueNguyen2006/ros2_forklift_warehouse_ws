import importlib.util
import os
import sqlite3
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xacro

from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

NAV_BASE_OFFSET_X = 0.55
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
REAR_STEER_CAMERA_MOUNT_X = 1.40
REAR_STEER_CAMERA_MOUNT_Z = 2.30


def is_valid_rtabmap_db(path: str) -> bool:
    if not (os.path.isfile(path) and os.path.getsize(path) > 4096):
        return False

    connection = None
    try:
        connection = sqlite3.connect(path)
        cursor = connection.cursor()
        cursor.execute("SELECT count(*) FROM Node")
        node_count = int(cursor.fetchone()[0])
        cursor.execute("SELECT count(*) FROM Word")
        word_count = int(cursor.fetchone()[0])
    except sqlite3.Error:
        return False
    finally:
        try:
            connection.close()
        except Exception:
            pass

    return node_count > 0 and word_count > 0


def select_default_database_path(visual_dir: str) -> str:
    candidates = [
        os.environ.get("WAREHOUSE_RTABMAP_DB", ""),
        os.path.join(
            os.path.expanduser("~"),
            "ros2_forklift_warehouse_artifacts",
            "results",
            "test_mapping.db",
        ),
        os.path.join(visual_dir, "maps", "warehouse_rtabmap.db"),
        os.path.join(
            os.path.expanduser("~"),
            "ros2_forklift_warehouse_ws",
            "src",
            "warehouse_visual_localization",
            "maps",
            "warehouse_rtabmap.db",
        ),
    ]

    for candidate in candidates:
        if candidate and is_valid_rtabmap_db(candidate):
            return candidate

    return os.path.join(visual_dir, "maps", "warehouse_rtabmap.db")


def load_baseline_launch_module(bringup_dir: str):
    source_path = os.path.join(bringup_dir, "launch", "warehouse_nav_baseline.launch.py")
    spec = importlib.util.spec_from_file_location(
        "forklift_nav_baseline_visual_import",
        source_path,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def configure_visual_planar_base(root, baseline_module) -> None:
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

    base_link_gazebo = baseline_module.ElementTree.SubElement(
        root,
        "gazebo",
        {"reference": "base_link"},
    )
    baseline_module.ElementTree.SubElement(base_link_gazebo, "gravity").text = "false"

    planar_move = baseline_module.ElementTree.SubElement(root, "gazebo")
    plugin = baseline_module.ElementTree.SubElement(
        planar_move,
        "plugin",
        {"name": "planar_move", "filename": "libgazebo_ros_planar_move.so"},
    )
    ros = baseline_module.ElementTree.SubElement(plugin, "ros")
    baseline_module.ElementTree.SubElement(ros, "remapping").text = "cmd_vel:=/cmd_vel"
    baseline_module.ElementTree.SubElement(ros, "remapping").text = (
        "odom:=/sim_wheel_odom"
    )
    baseline_module.ElementTree.SubElement(plugin, "update_rate").text = "100.0"
    baseline_module.ElementTree.SubElement(plugin, "publish_rate").text = "30.0"
    baseline_module.ElementTree.SubElement(plugin, "publish_odom").text = "true"
    baseline_module.ElementTree.SubElement(plugin, "publish_odom_tf").text = "false"
    baseline_module.ElementTree.SubElement(plugin, "odometry_frame").text = "odom"
    baseline_module.ElementTree.SubElement(plugin, "robot_base_frame").text = (
        "base_footprint"
    )
    baseline_module.ElementTree.SubElement(plugin, "covariance_x").text = "0.0001"
    baseline_module.ElementTree.SubElement(plugin, "covariance_y").text = "0.0001"
    baseline_module.ElementTree.SubElement(plugin, "covariance_yaw").text = "0.01"


def build_visual_robot_description(bringup_dir: str, forklift_robot_dir: str) -> str:
    baseline_module = load_baseline_launch_module(bringup_dir)
    original_configure_planar_base = baseline_module._configure_planar_base
    baseline_module._configure_planar_base = (
        lambda root: configure_visual_planar_base(root, baseline_module)
    )
    try:
        return baseline_module._build_baseline_robot_description(forklift_robot_dir)
    finally:
        baseline_module._configure_planar_base = original_configure_planar_base


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


def _ensure_visual_base_footprint(root: ElementTree.Element) -> None:
    if root.find("./link[@name='base_footprint']") is not None:
        return

    base_footprint = ElementTree.Element("link", {"name": "base_footprint"})
    base_footprint_joint = ElementTree.Element(
        "joint",
        {"name": "base_link_to_base_footprint", "type": "fixed"},
    )
    ElementTree.SubElement(base_footprint_joint, "parent", {"link": "base_link"})
    ElementTree.SubElement(base_footprint_joint, "child", {"link": "base_footprint"})
    ElementTree.SubElement(
        base_footprint_joint,
        "origin",
        {"xyz": f"{NAV_BASE_OFFSET_X} 0 0", "rpy": "0 0 0"},
    )

    root.append(base_footprint)
    root.append(base_footprint_joint)


def _disable_rear_steer_lidar_visualization(root: ElementTree.Element) -> None:
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


def _add_camera_sensor(
    root: ElementTree.Element,
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


def _add_rear_steer_camera_suite(root: ElementTree.Element) -> None:
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
    add_link("stereo_left_camera_link")
    add_link("stereo_left_camera_optical_link")
    add_link("stereo_right_camera_link")
    add_link("stereo_right_camera_optical_link")

    add_fixed_joint(
        "rgb_camera_joint",
        "cabin_link",
        "rgb_camera_link",
        f"{REAR_STEER_CAMERA_MOUNT_X} 0.0 {REAR_STEER_CAMERA_MOUNT_Z}",
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
        "cabin_link",
        "depth_camera_link",
        f"{REAR_STEER_CAMERA_MOUNT_X + 0.03} 0.0 {REAR_STEER_CAMERA_MOUNT_Z - 0.08}",
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
        "cabin_link",
        "stereo_left_camera_link",
        f"{REAR_STEER_CAMERA_MOUNT_X - 0.02} {STEREO_BASELINE / 2.0} {REAR_STEER_CAMERA_MOUNT_Z - 0.02}",
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
        "cabin_link",
        "stereo_right_camera_link",
        f"{REAR_STEER_CAMERA_MOUNT_X - 0.02} -{STEREO_BASELINE / 2.0} {REAR_STEER_CAMERA_MOUNT_Z - 0.02}",
        "0 0 0",
    )
    add_fixed_joint(
        "stereo_right_camera_optical_joint",
        "stereo_right_camera_link",
        "stereo_right_camera_optical_link",
        "0 0 0",
        "-1.57079632679 0 -1.57079632679",
    )

    _add_camera_sensor(
        root,
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
    _add_camera_sensor(
        root,
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
    _add_camera_sensor(
        root,
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
    _add_camera_sensor(
        root,
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


def build_visual_rear_steer_robot_description(
    realistic_dir: str,
    controller_config_path: str,
) -> str:
    robot_description = xacro.process_file(
        os.path.join(realistic_dir, "urdf", "rear_steer_forklift.urdf.xacro")
    ).toxml()
    root = ElementTree.fromstring(robot_description)

    _ensure_visual_base_footprint(root)
    _disable_rear_steer_lidar_visualization(root)
    _remove_existing_camera_assets(root)
    _add_rear_steer_camera_suite(root)

    gazebo_control_plugin = root.find("./gazebo/plugin[@name='gazebo_ros2_control']")
    if gazebo_control_plugin is not None:
        parameters = gazebo_control_plugin.find("parameters")
        if parameters is None:
            parameters = ElementTree.SubElement(gazebo_control_plugin, "parameters")
        parameters.text = controller_config_path

    return ElementTree.tostring(root, encoding="unicode")


def get_common_paths():
    visual_dir = get_package_share_directory("warehouse_visual_localization")
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    forklift_robot_dir = get_package_share_directory("forklift_robot")
    realistic_dir = get_package_share_directory("forklift_description_realistic")
    ros_gazebo_plugins_prefix = get_package_prefix("gazebo_plugins")
    ros_gazebo_plugin_dir = os.path.join(ros_gazebo_plugins_prefix, "lib")

    return {
        "visual_dir": visual_dir,
        "bringup_dir": bringup_dir,
        "gazebo_ros_dir": gazebo_ros_dir,
        "forklift_robot_dir": forklift_robot_dir,
        "realistic_dir": realistic_dir,
        "ros_gazebo_plugin_dir": ros_gazebo_plugin_dir,
    }


def make_runtime_env_actions(bringup_dir: str, ros_gazebo_plugin_dir: str):
    libgl_software = PythonExpression(
        [
            "'1' if '",
            LaunchConfiguration("force_software_rendering"),
            "' == 'true' or '",
            LaunchConfiguration("gui"),
            "' == 'false' or '",
            LaunchConfiguration("headless"),
            "' == 'true' else '0'",
        ]
    )
    return [
        SetEnvironmentVariable(
            "MESA_D3D12_DEFAULT_ADAPTER_NAME",
            LaunchConfiguration("mesa_adapter_name"),
        ),
        SetEnvironmentVariable(
            "AMENT_PREFIX_PATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                ":",
                EnvironmentVariable("AMENT_PREFIX_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "CMAKE_PREFIX_PATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                ":",
                EnvironmentVariable("CMAKE_PREFIX_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "COLCON_PREFIX_PATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                ":",
                EnvironmentVariable("COLCON_PREFIX_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", libgl_software),
        SetEnvironmentVariable(
            "QT_QPA_PLATFORM",
            EnvironmentVariable("QT_QPA_PLATFORM", default_value="xcb"),
        ),
        SetEnvironmentVariable(
            "GDK_BACKEND",
            EnvironmentVariable("GDK_BACKEND", default_value="x11"),
        ),
        SetEnvironmentVariable(
            "QT_OPENGL",
            PythonExpression(
                [
                    "'software' if '",
                    LaunchConfiguration("force_software_rendering"),
                    "' == 'true' else 'desktop'",
                ]
            ),
        ),
        SetEnvironmentVariable(
            "OGRE_RTT_MODE",
            EnvironmentVariable("OGRE_RTT_MODE", default_value="Copy"),
        ),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            [
                os.path.join(bringup_dir, "models"),
                ":",
                "/usr/share/gazebo-11/models",
                ":",
                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "GAZEBO_PLUGIN_PATH",
            [
                ros_gazebo_plugin_dir,
                ":",
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
                ":",
                EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "GAZEBO_RESOURCE_PATH",
            [
                "/usr/share/gazebo-11",
                ":",
                EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                "/lib:",
                LaunchConfiguration("rtabmap_prefix"),
                "/lib/x86_64-linux-gnu:",
                ros_gazebo_plugin_dir,
                ":",
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
                ":",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "OGRE_RESOURCE_PATH",
            [
                "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
                ":",
                EnvironmentVariable("OGRE_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "PATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                "/bin:",
                EnvironmentVariable("PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "PYTHONPATH",
            [
                LaunchConfiguration("rtabmap_prefix"),
                "/lib/python3.10/site-packages:",
                EnvironmentVariable("PYTHONPATH", default_value=""),
            ],
        ),
    ]
