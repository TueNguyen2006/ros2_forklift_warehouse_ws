import numpy as np

from warehouse_visual_localization.core.cbf_filter import filter_linear_cbf_command


def test_safe_nominal_command_is_preserved():
    result = filter_linear_cbf_command([0.2, 0.0], [[1.0, 0.0]], [0.5], [-1, -1], [1, 1])
    assert result.feasible
    assert np.allclose(result.safe_command, [0.2, 0.0])


def test_unsafe_command_is_projected_and_bounded():
    result = filter_linear_cbf_command([0.9, 0.5], [[1.0, 0.0]], [0.3], [-0.5, -0.2], [0.5, 0.2])
    assert result.feasible
    assert result.safe_command[0] <= 0.3 + 1.0e-9
    assert np.all(result.safe_command <= [0.5, 0.2])


def test_infeasible_constraint_falls_back_to_stop():
    result = filter_linear_cbf_command([0.2], [[1.0]], [-1.0], [-0.5], [0.5])
    assert not result.feasible
    assert result.fallback_used
    assert result.safe_command[0] == 0.0
