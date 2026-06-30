import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("warehouse_visual_localization")
    params_file = os.path.join(package_dir, "config", "rtabmap_rgbd.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("base_frame_id", default_value="base_footprint"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("approx_sync", default_value="true"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
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
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                parameters=[
                    params_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                        "frame_id": LaunchConfiguration("base_frame_id"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                        "approx_sync": ParameterValue(
                            LaunchConfiguration("approx_sync"), value_type=bool
                        ),
                        "Reg/Force3DoF": ParameterValue("true", value_type=str),
                        "Vis/MinInliers": ParameterValue("8", value_type=str),
                        "Vis/MinDepth": ParameterValue("0.20", value_type=str),
                        "Vis/MaxDepth": ParameterValue("8.0", value_type=str),
                        "Kp/MaxFeatures": ParameterValue("2000", value_type=str),
                        "Odom/GuessMotion": ParameterValue("true", value_type=str),
                        "OdomF2M/MaxSize": ParameterValue("1400", value_type=str),
                        "Odom/ResetCountdown": ParameterValue("0", value_type=str),
                        "Odom/ImageBufferSize": ParameterValue("1", value_type=str),
                        "GFTT/MinDistance": ParameterValue("4", value_type=str),
                    },
                ],
                remappings=[
                    ("rgb/image", "/depth_camera/image_raw"),
                    ("depth/image", "/depth_camera/depth/image_raw"),
                    ("rgb/camera_info", "/depth_camera/camera_info"),
                    ("odom", LaunchConfiguration("odom_topic")),
                ],
            ),
        ]
    )
