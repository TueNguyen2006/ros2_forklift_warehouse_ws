import math

from forklift_control.kinematics import ForkliftKinematicConfig, integrate_odometry


def test_straight_motion_has_small_yaw():
    cfg = ForkliftKinematicConfig()
    state = (0.0, 0.0, 0.0)

    for _ in range(50):
        state = integrate_odometry(state, 1.0 / cfg.wheel_radius, 0.0, 0.02, cfg)

    x, y, yaw = state
    assert x > 0.9
    assert abs(y) < 1e-6
    assert abs(yaw) < 1e-6
    assert math.isfinite(x)
