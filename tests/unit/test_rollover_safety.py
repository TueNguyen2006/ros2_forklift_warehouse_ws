from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover


SUPPORT = [(-0.6, -0.5), (0.6, -0.5), (0.6, 0.5), (-0.6, 0.5)]


def test_higher_cog_has_less_lateral_margin():
    low = evaluate_rollover(0.5, 0.4, 0.0, SUPPORT, RolloverConfig(vehicle_cog_height=0.35, safety_factor=0.8))
    high = evaluate_rollover(0.5, 0.4, 0.0, SUPPORT, RolloverConfig(vehicle_cog_height=0.8, safety_factor=0.8))
    assert high.lateral_acceleration_limit < low.lateral_acceleration_limit
    assert high.utilization_ratio > low.utilization_ratio


def test_zmp_outside_support_is_unsafe():
    result = evaluate_rollover(1.0, 6.0, 0.0, SUPPORT, RolloverConfig(vehicle_cog_height=1.0, safety_factor=1.0))
    assert not result.safe
    assert result.reason_code in ("lateral_acceleration_limit", "zmp_outside_support")


def test_invalid_track_width_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        evaluate_rollover(0.2, 0.1, 0.0, SUPPORT, RolloverConfig(track_width=0.0))
