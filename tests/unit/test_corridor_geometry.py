from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, evaluate_corridor
from warehouse_visual_localization.core.types import Pose2D


POINTS = [(0.5, 0.3), (0.5, -0.3), (-0.5, -0.3), (-0.5, 0.3)]
LEFT = CorridorBoundary((0.0, 1.0), 1.0)
RIGHT = CorridorBoundary((0.0, 1.0), -1.0)


def test_centered_footprint_is_feasible():
    result = evaluate_corridor(POINTS, Pose2D(), LEFT, RIGHT)
    assert result.feasible
    assert result.minimum_margin > 0.0


def test_corner_touch_and_outside_are_detected():
    touch = evaluate_corridor(POINTS, Pose2D(y=0.7), LEFT, RIGHT)
    outside = evaluate_corridor(POINTS, Pose2D(y=0.71), LEFT, RIGHT)
    assert touch.feasible
    assert not outside.feasible
    assert outside.maximum_violation > 0.0


def test_rear_swing_can_violate_while_reference_is_inside():
    result = evaluate_corridor(POINTS, Pose2D(y=0.62, yaw=0.65), LEFT, RIGHT)
    assert not result.feasible


def test_invalid_polygon_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        evaluate_corridor([(0.0, 0.0), (1.0, 0.0)], Pose2D(), LEFT, RIGHT)
