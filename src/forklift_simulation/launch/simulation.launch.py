import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    simulation_dir = get_package_share_directory("forklift_simulation")
    planar_launch = os.path.join(simulation_dir, "launch", "planar_simulation.launch.py")
    physics_launch = os.path.join(simulation_dir, "launch", "physics_simulation.launch.py")
    default_planar_world = os.path.join(simulation_dir, "worlds", "small_warehouse_open_top.world")
    default_physics_world = os.path.join(simulation_dir, "worlds", "physics_floor.world")

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="planar"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz", default_value=LaunchConfiguration("use_rviz")),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_planar_world),
            DeclareLaunchArgument("physics_world", default_value=default_physics_world),
            DeclareLaunchArgument("spawn_x", default_value="-2.3"),
            DeclareLaunchArgument("spawn_y", default_value="-2.3"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("physics_spawn_z", default_value="0.30"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planar_launch),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("simulation_mode"), "' == 'planar'"])),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(physics_launch),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("simulation_mode"), "' == 'physics'"])),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("physics_world"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("physics_spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
        ]
    )
