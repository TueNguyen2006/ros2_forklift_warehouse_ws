from pathlib import Path


def test_planar_mode_remains_default_and_uses_existing_visual_stack():
    simulation_launch = Path("src/forklift_simulation/launch/simulation.launch.py").read_text()
    planar_launch = Path("src/forklift_simulation/launch/planar_simulation.launch.py").read_text()
    launch_common = Path("src/warehouse_visual_localization/warehouse_visual_localization/launch_common.py").read_text()

    assert "default_value=\"planar\"" in simulation_launch
    assert "sim_sensors.launch.py" in planar_launch
    assert "libgazebo_ros_planar_move.so" in launch_common
