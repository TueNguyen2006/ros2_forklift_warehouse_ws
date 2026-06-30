import os
import sqlite3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_valid_rtabmap_db(path: str) -> bool:
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


def _select_default_database_path(package_dir: str) -> str:
    candidates = [
        os.environ.get("WAREHOUSE_RTABMAP_DB", ""),
        os.path.join(
            os.path.expanduser("~"),
            "ros2_forklift_warehouse_artifacts",
            "results",
            "test_mapping.db",
        ),
        os.path.join(package_dir, "maps", "warehouse_rtabmap.db"),
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
        if candidate and _is_valid_rtabmap_db(candidate):
            return candidate

    return os.path.join(package_dir, "maps", "warehouse_rtabmap.db")


def _validate_database_path(context, *_, **__):
    if (
        LaunchConfiguration("localization").perform(context).strip().lower()
        not in {"1", "true", "yes", "on"}
    ):
        return []

    database_path = LaunchConfiguration("database_path").perform(context)
    if _is_valid_rtabmap_db(database_path):
        return []

    raise RuntimeError(
        "RTAB-Map localization requested, but database is not valid: "
        f"{database_path}. Expected a database with at least one Node and one Word. "
        "Rebuild the mapping database before running localization:=true."
    )


def generate_launch_description():
    package_dir = get_package_share_directory("warehouse_visual_localization")
    params_file = os.path.join(package_dir, "config", "rtabmap_rgbd.yaml")
    default_db = _select_default_database_path(package_dir)

    localization_arg = LaunchConfiguration("localization")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("localization", default_value="true"),
            DeclareLaunchArgument("initial_pose", default_value=""),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("base_frame_id", default_value="base_footprint"),
            DeclareLaunchArgument(
                "rtabmap_prefix",
                default_value=os.path.expanduser("~/ros2_local_overlay/opt/ros/humble"),
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
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    LaunchConfiguration("rtabmap_prefix"),
                    "/lib:",
                    LaunchConfiguration("rtabmap_prefix"),
                    "/lib/x86_64-linux-gnu:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
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
            OpaqueFunction(function=_validate_database_path),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                parameters=[
                    params_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "database_path": LaunchConfiguration("database_path"),
                        "initial_pose": LaunchConfiguration("initial_pose"),
                        "frame_id": LaunchConfiguration("base_frame_id"),
                        "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                        "approx_sync": True,
                        "subscribe_rgb": True,
                        "subscribe_depth": True,
                        "subscribe_rgbd": False,
                        "Mem/IncrementalMemory": ParameterValue(
                            PythonExpression(
                                [
                                    "'false' if '",
                                    localization_arg,
                                    "' == 'true' else 'true'",
                                ]
                            ),
                            value_type=str,
                        ),
                        "Mem/InitWMWithAllNodes": ParameterValue(
                            PythonExpression(
                                [
                                    "'true' if '",
                                    localization_arg,
                                    "' == 'true' else 'false'",
                                ]
                            ),
                            value_type=str,
                        ),
                        "Reg/Force3DoF": ParameterValue("true", value_type=str),
                        "RGBD/OptimizeMaxError": ParameterValue("3.0", value_type=str),
                        "RGBD/AngularUpdate": ParameterValue("0.03", value_type=str),
                        "RGBD/LinearUpdate": ParameterValue("0.03", value_type=str),
                        "RGBD/ProximityBySpace": ParameterValue("false", value_type=str),
                        "RGBD/NeighborLinkRefining": ParameterValue(
                            "true", value_type=str
                        ),
                        "Vis/MinInliers": ParameterValue("8", value_type=str),
                        "Vis/MinDepth": ParameterValue("0.20", value_type=str),
                        "Vis/MaxDepth": ParameterValue("8.0", value_type=str),
                        "Kp/MaxFeatures": ParameterValue("1600", value_type=str),
                        "Odom/GuessMotion": ParameterValue("true", value_type=str),
                        "Mem/NotLinkedNodesKept": ParameterValue(
                            "false", value_type=str
                        ),
                    },
                ],
                remappings=[
                    ("rgb/image", "/depth_camera/image_raw"),
                    ("depth/image", "/depth_camera/depth/image_raw"),
                    ("rgb/camera_info", "/depth_camera/camera_info"),
                    ("odom", "/odom"),
                    ("map", "/rtabmap/localization_map"),
                ],
            ),
        ]
    )
