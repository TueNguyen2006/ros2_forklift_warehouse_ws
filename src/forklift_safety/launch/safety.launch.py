from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("stability_input_topic", default_value="cmd_vel_smoothed"),
            DeclareLaunchArgument("stability_output_topic", default_value="cmd_vel_stability"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("profile_config_path", default_value=""),
            DeclareLaunchArgument("load_profile", default_value="EMPTY"),
            DeclareLaunchArgument("enable_odom_to_imu", default_value="false"),
            Node(
                package="forklift_safety",
                executable="stability_guard",
                name="stability_guard",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": LaunchConfiguration("stability_input_topic"),
                        "output_topic": LaunchConfiguration("stability_output_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "profile_config_path": LaunchConfiguration("profile_config_path"),
                        "load_profile": LaunchConfiguration("load_profile"),
                    }
                ],
            ),
        ]
    )
