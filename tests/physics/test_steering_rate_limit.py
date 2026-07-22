from forklift_control.actuator_limits import ActuatorCommand, ActuatorLimitConfig, limit_command


def test_steering_rate_limit_caps_angle_delta():
    cfg = ActuatorLimitConfig(max_steering_rate=0.5, max_steering_angle=0.55)
    previous = ActuatorCommand(speed=0.0, steering_angle=0.0)
    target = ActuatorCommand(speed=0.0, steering_angle=0.55)

    limited = limit_command(previous, target, dt=0.1, config=cfg)

    assert limited.steering_angle == 0.05
