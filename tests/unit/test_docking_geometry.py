import math

from warehouse_visual_localization.core.docking_geometry import DockingConfig, evaluate_docking


def test_perfect_alignment_is_feasible():
    assert evaluate_docking(0.0, 0.0).docking_feasible


def test_two_degree_heading_moves_fork_tip_centimetres():
    result = evaluate_docking(0.0, math.radians(2.0), DockingConfig(insertion_length=1.0, pocket_width=0.08, clearance_margin=0.01))
    assert abs(result.exact_tip_error) > 0.03
    assert not result.docking_feasible


def test_invalid_geometry_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        evaluate_docking(0.0, 0.0, DockingConfig(insertion_length=0.0))
