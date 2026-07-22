from forklift_control.kinematics import ForkliftKinematicConfig, yaw_rate_from_state


def test_reverse_turn_flips_yaw_direction():
    cfg = ForkliftKinematicConfig(steering_axle="rear")
    steering = -0.3

    forward_yaw = yaw_rate_from_state(0.5, steering, cfg)
    reverse_yaw = yaw_rate_from_state(-0.5, steering, cfg)

    assert forward_yaw == -reverse_yaw
