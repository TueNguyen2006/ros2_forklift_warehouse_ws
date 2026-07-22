# Two-Layer Simulation Architecture

This repository now separates simulation into two modes:

```text
simulation_mode:=planar
simulation_mode:=physics
```

`planar` remains the default and preserves the current Nav2/RTAB-Map/EKF workflow. `physics` is a separate wheel-contact model intended for controller, dynamics, and RL development. It must not silently fall back to planar behavior.

## Current Planar Pipeline

```text
Nav2 controller_server
  -> cmd_vel_nav
  -> nav2_velocity_smoother
  -> /visual_nav/cmd_vel_request
  -> planar_motion_guard
  -> /cmd_vel
  -> libgazebo_ros_planar_move.so
  -> /sim_wheel_odom

RGB-D camera
  -> RTAB-Map RGB-D odometry
  -> /visual_odom

/sim_wheel_odom + /visual_odom
  -> robot_localization EKF
  -> /odom and odom -> base_footprint
```

The planar model is intentionally fast and stable for Nav2, RViz, localization, and debugging.

Planar mode keeps using `small_warehouse_open_top.world` by default.

## Files That Strip `ros2_control`

- `src/warehouse_visual_localization/warehouse_visual_localization/launch_common.py`
  - `configure_visual_planar_base()` removes `<ros2_control>`.
  - It removes Gazebo plugin blocks containing `libgazebo_ros_diff_drive.so`.
  - It removes Gazebo plugin blocks containing `libgazebo_ros2_control.so`.
- `src/forklift_simulation/launch/warehouse_nav_baseline_legacy_helpers.launch.py`
  - `_configure_planar_base()` performs equivalent stripping for the legacy helper path.

## Files That Inject Planar Move

- `src/warehouse_visual_localization/warehouse_visual_localization/launch_common.py`
  - Adds `libgazebo_ros_planar_move.so`.
  - Remaps `cmd_vel:=/cmd_vel`.
  - Remaps `odom:=/sim_wheel_odom`.
- `src/forklift_simulation/launch/warehouse_nav_baseline_legacy_helpers.launch.py`
  - Adds the same planar move plugin for compatibility helper use.

## Existing Joints

The copied legacy description already contains:

- `left_wheel_joint`
- `right_wheel_joint`
- `caster_wheel_joint`
- `fork_base_joint`
- fixed chassis, mast, fork, lidar, and camera joints

The legacy `ros2_control.xacro` exposes velocity interfaces for `left_wheel_joint` and `right_wheel_joint`, but the active planar visual stack removes this in favor of the planar Gazebo plugin.

## Missing Joints For Real Forklift Steering

The legacy model does not separate steering and wheel rotation. Physics mode requires:

```text
chassis -> steering link        steering joint
steering link -> wheel link     wheel rotation joint
```

The new physics model introduces:

- `left_rear_steering_joint`
- `right_rear_steering_joint`
- `left_rear_wheel_rotation_joint`
- `right_rear_wheel_rotation_joint`
- `left_front_wheel_rotation_joint`
- `right_front_wheel_rotation_joint`

The default assumption is:

```text
steering_axle:=rear
drive_axle:=front
physics_spawn_z:=0.30
```

This matches a conservative forklift-like rear-steer layout while keeping drive commands on the front axle. It is an initial model assumption, not a measured vehicle fact.

Physics mode uses `physics_floor.world` by default. The warehouse mesh ground is not used as the default physics contact surface because its copied AWS model has very high friction and no slip, which can destabilize wheel contact while the wheel model is still being validated.

## Reusable Parts

- Warehouse worlds and Gazebo model assets from `forklift_simulation`.
- Nav2 maps, behavior trees, and parameters from `forklift_navigation`.
- RGB-D camera, lidar topics, RTAB-Map odometry, EKF config, and visual localization nodes.
- Controller research code in `warehouse_visual_localization.core.controller`.
- Kinematics, friction, CBF, slip, and controller labs/tests as references for new physics tests.

## Unknown Mechanical Assumptions

The following must be measured or confirmed before claiming sim-to-real validity:

- exact steering axle and drive axle;
- wheelbase;
- track width;
- wheel radius under load;
- steering limits and steering rate;
- motor velocity and acceleration limits;
- brake/deceleration limits;
- chassis mass and center of mass;
- payload mass and payload center of mass;
- tire-ground friction and slip parameters;
- contact stiffness and damping;
- steering backlash and control delay.

## Physics Pipeline

```text
/cmd_vel
  -> ackermann_command_adapter
  -> /forklift/control/target
  -> steering_position_controller + drive_velocity_controller
  -> steering joints + drive wheel joints
  -> Gazebo contact physics
  -> /joint_states
  -> wheel_odometry
  -> /wheel/odom
```

Ground truth is kept separate:

```text
Gazebo entity state -> /ground_truth/odom
```

It is intended for metrics, debugging, and optional RL reward instrumentation, not as default observation or wheel odometry.

Physics mode disables the depth camera by default in the new Xacro because Gazebo Classic camera rendering can crash in headless WSL smoke tests. Planar mode keeps the existing camera/depth/lidar/RTAB-Map pipeline unchanged.
