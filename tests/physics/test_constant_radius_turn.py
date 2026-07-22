import math

import pytest

from forklift_control.kinematics import ForkliftKinematicConfig, yaw_rate_from_state


def test_constant_radius_turn_matches_kinematic_radius():
    cfg = ForkliftKinematicConfig(wheelbase=1.35, steering_axle="rear")
    speed = 0.6
    steering = -0.35

    yaw_rate = yaw_rate_from_state(speed, steering, cfg)
    measured_radius = abs(speed / yaw_rate)
    theoretical_radius = abs(cfg.wheelbase / math.tan(steering))

    assert measured_radius == pytest.approx(theoretical_radius, rel=1e-6)
