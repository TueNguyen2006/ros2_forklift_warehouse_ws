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
    workspace_dir = os.environ.get("VISUAL_LOCALIZATION_WORKSPACE_DIR", "")
    candidates = [
        os.environ.get("WAREHOUSE_RTABMAP_DB", ""),
        os.path.join(
            os.path.expanduser("~"),
            "ros2_forklift_warehouse_artifacts",
            "results",
            "test_mapping.db",
        ),
        os.path.join(visual_dir, "maps", "warehouse_rtabmap.db"),
    ]
    if workspace_dir:
        candidates.append(
            os.path.join(
                workspace_dir,
                "src",
                "warehouse_visual_localization",
                "maps",
                "warehouse_rtabmap.db",
            )
        )

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


def get_common_paths():
    visual_dir = get_package_share_directory("warehouse_visual_localization")
    bringup_dir = get_package_share_directory("forklift_nav_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    forklift_robot_dir = get_package_share_directory("forklift_robot")
    ros_gazebo_plugins_prefix = get_package_prefix("gazebo_plugins")
    ros_gazebo_plugin_dir = os.path.join(ros_gazebo_plugins_prefix, "lib")

    return {
        "visual_dir": visual_dir,
        "bringup_dir": bringup_dir,
        "gazebo_ros_dir": gazebo_ros_dir,
        "forklift_robot_dir": forklift_robot_dir,
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
