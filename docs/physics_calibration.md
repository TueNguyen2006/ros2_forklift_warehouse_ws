# Physics Calibration

Calibrate physics mode incrementally. Do not tune one large parameter set all at once.

## Wheel Radius

Command low constant wheel velocity on a high-friction floor. Measure traveled distance and compare with:

```text
distance = wheel_radius * wheel_angular_velocity * time
```

Adjust radius until straight-line distance matches.

## Wheelbase

Command a small constant steering angle and low speed. Estimate turning radius from the path:

```text
turning_radius = wheelbase / tan(steering_angle)
```

For rear steering, yaw sign is inverted relative to front steering.

## Steering Limits

Increase steering commands slowly until the joint reaches its mechanical bound. Set:

```text
max_steering_angle
max_steering_rate
```

from measured behavior.

## Acceleration And Braking

Apply step speed commands and measure velocity slope. Configure:

```text
max_accel
max_decel
max_speed
max_reverse_speed
```

from the observed envelope.

## Friction

Run straight braking and constant-radius turns on normal and low-friction floors. Tune longitudinal and lateral friction so slip increases on low-friction floor without making the normal floor artificially perfect.

## Payload And Center Of Mass

Repeat acceleration, braking, and turning tests with known payload masses and positions. Update:

```text
payload_mass
vehicle center of mass
rollover safety parameters
```

only after measurements are available.
