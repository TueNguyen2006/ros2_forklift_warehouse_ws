# RL Environment

`forklift_rl` provides a Gymnasium-compatible environment skeleton for physics-mode training.

Initial action:

```text
[target_speed, target_steering_angle]
```

Initial observation fields:

- relative goal x/y;
- heading error;
- cross-track error;
- current velocity;
- current steering angle;
- previous action;
- downsampled lidar ranges.

Camera observations are intentionally excluded from the first version.

The environment contains a `SimulatorStepper` abstraction so deterministic stepping is centralized. The current implementation is a no-op smoke-test boundary until the active Gazebo Classic installation exposes a reliable synchronous step service or plugin. Training code must not spread `time.sleep()` calls through the environment.

Domain randomization config lives at:

```text
src/forklift_rl/config/domain_randomization.yaml
```

It is disabled by default and supports seeded sampling for reproducibility.
