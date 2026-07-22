"""Headless integration checks for optional canonical launch wiring."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_launch_keeps_default_direct_command_path_and_declares_opt_ins():
    launch = (ROOT / "src/warehouse_visual_localization/launch/nav_with_estimated_pose.launch.py").read_text(encoding="utf-8")
    for argument in ("canonical_monitor", "curvature_speed_limit", "corridor_monitor", "rollover_monitor", "human_ssm", "cbf_filter", "benchmark_logger"):
        assert f'DeclareLaunchArgument("{argument}", default_value="false")' in launch
    assert '"input_topic": PythonExpression' in launch
    assert "'/canonical/cmd_vel_filtered'" in launch
    assert "'/visual_nav/cmd_vel_request'" in launch


def test_ros_adapters_import_core_formulas_without_copies():
    nodes = ROOT / "src/warehouse_visual_localization/warehouse_visual_localization/ros_nodes"
    monitor = (nodes / "canonical_trajectory_monitor.py").read_text(encoding="utf-8")
    command_filter = (nodes / "canonical_command_filter.py").read_text(encoding="utf-8")
    assert "from warehouse_visual_localization.core.curvature_speed import" in monitor
    assert "from warehouse_visual_localization.core.cbf_filter import" in command_filter
    assert '"/cmd_vel"' not in command_filter


def test_modular_launches_route_through_new_packages():
    bringup = (ROOT / "src/forklift_bringup/launch/bringup.launch.py").read_text(encoding="utf-8")
    visual = (ROOT / "src/warehouse_visual_localization/launch/nav_with_estimated_pose.launch.py").read_text(encoding="utf-8")
    common = (ROOT / "src/warehouse_visual_localization/warehouse_visual_localization/launch_common.py").read_text(encoding="utf-8")

    assert 'get_package_share_directory("forklift_simulation")' in bringup
    assert 'get_package_share_directory("forklift_navigation")' in bringup
    assert 'from forklift_simulation.world_map_generator import' in visual
    assert 'package="forklift_navigation"' in visual
    assert 'get_package_share_directory("forklift_description")' in common
