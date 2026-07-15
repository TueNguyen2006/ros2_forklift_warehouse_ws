# Core Algorithms

This package contains simulator-independent algorithms shared by:

```text
tests/standalone
ROS nodes
Gazebo integration
```

Modules:

- `vehicle_model.py`: 2D four-wheel forklift model with rear/front steering options.
- `controller.py`: standalone trajectory tracking controller.
- `safety.py`: CBF-style safety filter.
- `friction.py`: slip and friction force saturation helpers.
- `profiles.py`: velocity and acceleration profiles.
- `localization.py`: lightweight 2D EKF for standalone localization tests.
- `command_shaper.py`: command shaping shared by ROS `planar_motion_guard.py`.

Design rule:

```text
Fix formulas here once.
All standalone tests and ROS/Gazebo adapters pick up the change.
```

