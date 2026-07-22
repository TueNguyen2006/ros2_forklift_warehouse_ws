from forklift_control.actuator_limits import ActuatorCommand, ActuatorLimitConfig, limit_command


def test_acceleration_and_braking_are_limited():
    cfg = ActuatorLimitConfig(max_accel=0.4, max_decel=0.8, max_speed=2.0)

    accelerated = limit_command(
        ActuatorCommand(speed=0.0, steering_angle=0.0),
        ActuatorCommand(speed=2.0, steering_angle=0.0),
        dt=0.5,
        config=cfg,
    )
    braked = limit_command(
        ActuatorCommand(speed=1.0, steering_angle=0.0),
        ActuatorCommand(speed=-1.0, steering_angle=0.0),
        dt=0.5,
        config=cfg,
    )

    assert accelerated.speed == 0.2
    assert braked.speed == 0.6
