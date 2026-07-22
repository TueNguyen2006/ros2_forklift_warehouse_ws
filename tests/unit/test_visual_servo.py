import numpy as np

from warehouse_visual_localization.core.visual_servo import ibvs_command, planar_pbvs_command


def test_zero_ibvs_error_produces_zero_command():
    result = ibvs_command([0.2, -0.1], [0.2, -0.1], [[1, 0], [0, 1]], 1.0)
    assert result.valid
    assert np.allclose(result.camera_twist, 0.0)


def test_ibvs_command_reduces_synthetic_linear_error():
    result = ibvs_command([1.0, -1.0], [0.0, 0.0], [[1, 0], [0, 1]], 0.5)
    next_feature = np.array([1.0, -1.0]) + result.camera_twist
    assert np.linalg.norm(next_feature) < np.linalg.norm([1.0, -1.0])


def test_rank_deficient_ibvs_and_planar_yaw_wrap_do_not_crash():
    result = ibvs_command([1.0, 1.0], [0.0, 0.0], [[1, 0], [2, 0]], 1.0)
    assert result.rank == 1
    command = planar_pbvs_command([0, 0, 3.13], [0, 0, -3.13])
    assert abs(command.error[2]) < 0.1


def test_invalid_ibvs_shape_returns_safe_diagnostic():
    result = ibvs_command([1.0], [0.0, 1.0], [[1.0]], 1.0)
    assert not result.valid
    assert result.reason == "invalid_input"
