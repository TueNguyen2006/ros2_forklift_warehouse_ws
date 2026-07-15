# Interactive Algorithm Labs

These labs open lightweight Matplotlib windows. They do not launch Gazebo, RViz,
or Nav2. Each lab imports `warehouse_visual_localization.core`, so ROS, Gazebo
adapters, batch tests, and interactive labs share the same formulas.

Professional layout:

```text
tests/interactive/
├── common.py
├── run_interactive_smoke.sh
├── *_lab.py                  # Backward-compatible wrappers
└── labs/
    ├── kinematics_dynamics/
    │   ├── app.py
    │   ├── README.md
    │   └── KINEMATICS_REAR_STEER_FORKLIFT.md
    ├── cbf_safety/
    │   ├── app.py
    │   └── README.md
    ├── controller_tracking/
    │   ├── app.py
    │   └── README.md
    ├── slip_friction/
    ├── velocity_profile/
    └── localization/
```

New complex algorithms should go inside the relevant `labs/<name>/` folder, or
in a new sibling lab folder. Core formulas still belong in
`warehouse_visual_localization.core`.

Run from WSL:

```bash
cd /path/to/ros2_forklift_warehouse_ws
python3 tests/interactive/kinematics_dynamics_lab.py
python3 tests/interactive/cbf_safety_lab.py
python3 tests/interactive/controller_tracking_lab.py
python3 tests/interactive/slip_friction_lab.py
python3 tests/interactive/velocity_profile_lab.py
python3 tests/interactive/localization_lab.py
```

Equivalent direct lab app paths:

```bash
python3 tests/interactive/labs/kinematics_dynamics/app.py
python3 tests/interactive/labs/cbf_safety/app.py
python3 tests/interactive/labs/controller_tracking/app.py
```

Use the sliders and buttons on the right side of each window.

`kinematics_dynamics_lab.py` is wheel-level: it exposes individual wheel speeds
for `FL/FR/RL/RR`, front steering angle, rear steering angle, `dt`, and number of
integration steps. It solves the planar body twist from the four wheel
constraints and shows residual error when wheel commands are inconsistent.

`cbf_safety_lab.py` follows the octagonal-constraint animation style: the safe
set is a large octagon, the red background is unsafe, the forklift center of mass
is controlled toward a clicked reference point, and CBF projects the desired
velocity back into the safe set. Use `Play/Pause` for continuous animation,
`Step` for frame-by-frame inspection, and click in the plot to move the reference.

`controller_tracking_lab.py` can compare controllers with a dropdown-style
selector:

```text
Pure Pursuit
Stanley
MPPI
```

MPPI samples noisy short-horizon control sequences around a nominal sequence,
rolls them out through the same forklift model, scores path error, heading error,
goal distance, control effort, smoothness, and progress, then uses softmin
weights to update the nominal control sequence.
The right panel is grouped with separators: `Controller`, `Common`,
`Algorithm Parameters`, and `Actions`. The `Algorithm Parameters` section changes
content based on the selected controller instead of reserving separate space for
every algorithm.

For quick headless validation:

```bash
bash tests/interactive/run_interactive_smoke.sh
```
