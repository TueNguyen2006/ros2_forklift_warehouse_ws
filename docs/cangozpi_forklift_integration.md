# Cangozpi Forklift Integration Branch

This branch adds a direct launch path for the upstream forklift model from:

```text
https://github.com/cangozpi/ROS2-Forklift-Simulation
```

The repository already contains that project as a Git submodule at:

```text
src/third_party/ROS2-Forklift-Simulation
```

## Audit Result

The core files copied into `forklift_description` are byte-for-byte identical to the upstream `forklift_robot` package at submodule commit `ba74f76`:

- `forklift.urdf.xacro`
- `camera.xacro`
- `depth_camera.xacro`
- `lidar.xacro`
- `gazebo.xacro`
- `gazebo_colors.xacro`
- `gazebo_gravity_compensation_plugin.xacro`
- `inertial_macros.xacro`
- `ros2_control.xacro`
- `ros_gazebo_collision_detection_plugin.xacro`
- `template.urdf.xacro`
- `my_controllers.yaml`
- `forklift_rviz.rviz`
- `forklift_with_sensors.rviz`

The visibly unstable wheel-contact model reported during testing is not from that upstream URDF. It is from the newer custom files:

- `forklift_common.xacro`
- `forklift_wheels.xacro`
- `forklift_physics.xacro`
- `forklift_ros2_control.xacro`

## New Launch Mode

Use this mode to launch the upstream forklift model directly from the `forklift_robot` submodule package:

```bash
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=cangozpi
```

The launch processes `forklift_robot/forklift.urdf.xacro` directly. It applies two compatibility steps outside the submodule:

- replaces the upstream literal `length="laser_frame_length"` with `length="0.04"` so URDF parsing succeeds;
- removes `ros2_control` and collision-detection Gazebo plugins from this debug launch to avoid controller/plugin conflicts.

The upstream `libgazebo_ros_diff_drive.so` plugin is kept. This mode is therefore a cangozpi/diff-drive debug path, not the Ackermann/rear-steer physics model.

## Mode Boundaries

- `simulation_mode:=planar` keeps the current Nav2 visual-localization pipeline.
- `simulation_mode:=physics` keeps the custom Ackermann/rear-steer physics model.
- `simulation_mode:=cangozpi` uses the upstream forklift package directly for visual/model comparison.

Do not use the cangozpi mode to evaluate Ackermann steering behavior; the upstream model is diff-drive.

Expected non-fatal warnings:

- URDF material warnings for `blue` and `black`; Gazebo material tags still color the model.
- Gazebo Classic EOL warning.
- `timeout 45s ...` returns a nonzero exit because it intentionally stops a still-running simulation.
