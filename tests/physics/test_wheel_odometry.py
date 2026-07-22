from pathlib import Path


def test_wheel_odometry_does_not_use_ground_truth_service():
    source = Path("src/forklift_control/forklift_control/wheel_odometry.py").read_text()

    assert "GetEntityState" not in source
    assert "/ground_truth" not in source
    assert "/joint_states" in source
