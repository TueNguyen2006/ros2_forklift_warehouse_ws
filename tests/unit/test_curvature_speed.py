import numpy as np
import pytest

from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit, signed_curvature


def test_straight_path_has_zero_curvature_and_speed_cap():
    result = curvature_speed_limit(signed_curvature([[0, 0], [1, 0], [2, 0]]), CurvatureSpeedConfig(vehicle_speed_limit=0.4, lateral_acceleration_limit=0.3))
    assert np.allclose(result.curvature, 0.0)
    assert np.allclose(result.speed_limit, 0.4)


def test_speed_is_monotonic_with_absolute_curvature():
    result = curvature_speed_limit([0.0, 0.2, 1.0], CurvatureSpeedConfig(vehicle_speed_limit=2.0, lateral_acceleration_limit=0.4))
    assert result.speed_limit[0] >= result.speed_limit[1] >= result.speed_limit[2]


def test_duplicates_and_short_paths_are_finite():
    assert np.all(np.isfinite(signed_curvature([[0, 0], [0, 0], [1, 0], [2, 0]])))
    assert np.allclose(signed_curvature([[0, 0], [1, 0]]), 0.0)
    with pytest.raises(ValueError):
        curvature_speed_limit([float("nan")])
