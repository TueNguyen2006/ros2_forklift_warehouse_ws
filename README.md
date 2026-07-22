# Camera-Based Warehouse Navigation For Forklift

This repository has one main purpose:

Run a forklift in a Gazebo warehouse using simulated camera and wheel-odometry
sensors, then feed the estimated pose to Nav2 for path planning and path
following.

The main entry point is:

```bash
bash tools/run_visual_nav_manual.sh
```

The modular ROS 2 entry point is:

```bash
ros2 launch forklift_bringup bringup.launch.py
```

Low-level launch entry points:

```bash
ros2 launch forklift_simulation simulation.launch.py
ros2 launch forklift_navigation navigation.launch.py
ros2 launch forklift_safety safety.launch.py
ros2 launch forklift_evaluation evaluation.launch.py
```

Two-layer simulation entry points:

```bash
# Fast planar simulation, default/backward-compatible mode
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=planar

# Wheel-physics simulation for controller/dynamics/RL development
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=physics

# Direct upstream cangozpi forklift model, for model comparison/debug
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=cangozpi

# Optional low-friction physics floor
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=physics \
  physics_world:=$(ros2 pkg prefix forklift_simulation)/share/forklift_simulation/worlds/physics_low_friction.world

# Wheel-physics simulation with Gazebo GUI and RViz
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=physics \
  gui:=true \
  use_rviz:=true

# RL environment smoke launch, headless physics mode
ros2 launch forklift_rl rl_training_simulation.launch.py \
  headless:=true

# WSL smoke sequence for planar, physics, and RL launch paths
bash tools/smoke_two_layer_simulation.sh
```

Physics mode defaults to `physics_floor.world`, a simple Gazebo plane with explicit contact and friction parameters. Planar mode still defaults to `small_warehouse_open_top.world`. The warehouse ground mesh is intentionally not the default physics contact surface because its copied AWS collision model uses extreme friction and zero slip, which can make wheel-contact debugging look like wheel sinking, sticking, or lateral drift.

Build and test from the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rm -rf build install log
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For the direct upstream cangozpi model path, build the submodule packages first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --event-handlers console_direct+ \
  --packages-select ros_gazebo_plugins forklift_robot forklift_simulation
source install/setup.bash
ros2 launch forklift_simulation simulation.launch.py \
  simulation_mode:=cangozpi
```

Architecture notes:

- [ARCHITECTURE.md](ARCHITECTURE.md) describes package responsibilities.
- [MIGRATION.md](MIGRATION.md) lists old-to-new file locations.
- [DEPENDENCY_GRAPH.md](DEPENDENCY_GRAPH.md) documents the intended dependency direction.
- [docs/two_layer_simulation_architecture.md](docs/two_layer_simulation_architecture.md) documents planar vs physics simulation.
- [docs/cangozpi_forklift_integration.md](docs/cangozpi_forklift_integration.md) documents the upstream forklift-model branch mode.
- [docs/physics_model_assumptions.md](docs/physics_model_assumptions.md) lists current physics assumptions.
- [docs/physics_calibration.md](docs/physics_calibration.md) describes calibration steps before sim-to-real use.
- [docs/rl_environment.md](docs/rl_environment.md) describes the initial Gymnasium-compatible RL boundary.

## Goal

The original navigation stack worked when the robot pose came directly from the
simulator. This project replaces that ideal ground-truth pose with a sensor-based
pose pipeline:

```text
RGB-D camera
    -> RTAB-Map RGB-D visual odometry
    -> /visual_odom

simulated wheel odometry
    -> /sim_wheel_odom

/visual_odom + /sim_wheel_odom
    -> robot_localization EKF
    -> /odom and TF odom -> base_footprint

static warehouse map
    -> Nav2 global planning

Nav2
    -> /visual_nav/cmd_vel_request
    -> planar motion guard
    -> /cmd_vel
    -> Gazebo forklift motion
```

The navigation stack consumes the same kind of runtime interface used by a real
robot:

```text
map -> odom -> base_footprint
```

Ground-truth simulator pose is not used as the navigation pose source in this
visual navigation path.

## Current Design

The stable branch of this project intentionally uses a planar Gazebo drive model.
The planar model keeps the focus on the localization and navigation pipeline
instead of forklift tire/contact physics.

The current algorithmic stack is:

- `Gazebo Classic` simulates the warehouse, forklift body, cameras, lidar, and planar motion.
- `RGB-D camera` provides color and depth images for visual odometry.
- `RTAB-Map RGB-D odometry` estimates camera-based robot motion and publishes `/visual_odom`.
- `Gazebo planar move` publishes `/sim_wheel_odom`, representing encoder-like wheel odometry from the simulated drivetrain.
- `robot_localization EKF` fuses `/sim_wheel_odom` and `/visual_odom` into `/odom`.
- `Nav2` plans on the static warehouse occupancy map and follows the path using the fused odometry TF.
- `planar_motion_guard.py` smooths practical low-speed turning commands before sending `/cmd_vel` to Gazebo.
- `gazebo_goal_bridge.py` lets a goal selected in Gazebo be sent to Nav2.

## Why EKF Fusion Helps

Visual odometry and wheel odometry fail in different ways.

Visual odometry is useful because it observes motion from RGB-D camera data, so it
keeps the localization pipeline camera-based instead of simulator-truth-based.
However, visual odometry can become noisy or temporarily weak when texture,
lighting, depth quality, or camera motion is poor.

Simulated wheel odometry is smooth and high-rate. In this simulator it behaves
like a clean encoder source from the drivetrain. It is still odometry, so it is
not a global truth pose; it accumulates motion from the robot movement model.

The EKF combines them:

```text
prediction: wheel odometry gives smooth velocity and short-term motion
correction: visual odometry constrains camera-observed pose drift
output: stable /odom for Nav2 controller and costmaps
```

The result is much more stable path following than visual odometry alone.

## Computer Vision, Visual Odometry, And SLAM

Yes, this project does use computer vision. The camera part of the pipeline is
implemented through RTAB-Map RGB-D odometry.

The active visual input is:

```text
RGB image
Depth image
CameraInfo / calibration
```

These streams come from the simulated RGB-D camera in Gazebo. RTAB-Map then uses
computer-vision feature tracking and RGB-D geometric registration to estimate how
the camera moved between frames.

The simplified visual-odometry loop is:

```text
RGB-D frame at time t
    -> detect visual features / keypoints
    -> associate features with depth
    -> match features against previous frames / local map
    -> estimate relative 3D camera motion
    -> constrain motion to planar robot motion
    -> publish /visual_odom
```

In config, this is the `rtabmap_odom/rgbd_odometry` node. Important parameters
include:

```text
subscribe_rgb: true
subscribe_depth: true
Reg/Force3DoF: true
Vis/MinInliers: 8
Vis/MinDepth: 0.20
Vis/MaxDepth: 8.0
Kp/MaxFeatures: 2000
Odom/GuessMotion: true
```

`Reg/Force3DoF` is important because the forklift is treated as a ground robot:
the estimated visual motion should mainly be `(x, y, yaw)`, not free-flying
camera motion.

### What Is SLAM Here?

Strictly speaking, the default stable demo is not running full online SLAM as the
main source for map building. It is running:

```text
known static warehouse map
RGB-D visual odometry
wheel odometry
EKF fused odometry
Nav2 planning and control
```

So the default mode is best described as:

```text
camera-based odometry + map-based navigation
```

not:

```text
build a new map online while navigating
```

This is intentional. The current project goal is to replace simulator
ground-truth pose with sensor-derived pose while still using the known warehouse
map for Nav2 planning.

### RTAB-Map Localization Mode

The repository also contains RTAB-Map global localization support. That is the
SLAM-family component that can use a prebuilt RTAB-Map database to estimate:

```text
map -> odom
```

When enabled, the intended chain becomes:

```text
RGB-D camera
    -> RTAB-Map visual odometry
    -> odom -> base_footprint

prebuilt RTAB-Map database
    -> RTAB-Map localization
    -> map -> odom

Nav2
    -> consumes map -> odom -> base_footprint
```

This mode is controlled by:

```text
localization:=true
```

However, the stable manual demo currently keeps:

```text
localization:=false
```

That means the static warehouse occupancy map is still loaded for global
planning, but `map -> odom` is kept simple for the demo, while the moving robot
pose still comes from camera odometry plus wheel odometry through EKF.

### Difference Between The Terms

- `Computer vision`: the RGB-D image processing used to extract and match visual
  features.
- `Visual odometry`: the incremental camera-based motion estimate published as
  `/visual_odom`.
- `SLAM`: building or using a map while estimating robot pose. In this repo,
  full RTAB-Map global localization exists as an extension, but the stable path
  uses a known map and focuses on robust sensor-based odometry.
- `Navigation`: Nav2 uses the estimated pose and static map to plan and follow a
  path.

## Planner And Trajectory Following Algorithm

When a navigation goal is selected, this repository does not directly drive the
forklift toward the clicked point. The goal is converted into a global path first,
then a local trajectory follower repeatedly chooses short motion commands that
track that path.

The runtime loop is:

```text
Nav goal in map frame
    -> Nav2 behavior tree
    -> global planner: Smac Hybrid-A*
    -> global path /plan
    -> local controller: MPPI
    -> /visual_nav/cmd_vel_request
    -> planar motion guard
    -> /cmd_vel
    -> Gazebo forklift
```

### Global Planner

The visual navigation profile uses Nav2 `SmacPlannerHybrid` as the global
planner:

```text
planner_id: GridBased
plugin: nav2_smac_planner/SmacPlannerHybrid
motion_model_for_search: REEDS_SHEPP
minimum_turning_radius: 0.55
```

Algorithmically, this is a Hybrid-A* style planner. Instead of planning only on
2D grid cells `(x, y)`, it plans over a discretized vehicle state:

```text
state = (x, y, yaw)
```

That matters for a forklift-like robot because orientation is part of whether a
path is actually followable. A normal 2D grid planner can produce paths with
sharp corners that look valid on the map but are hard for a vehicle with turning
radius limits to follow. Smac Hybrid plans with heading bins, turning radius, and
motion primitives, so the path is closer to what the controller can execute.

The current planner allows forward and reverse motion through the Reeds-Shepp
search model. Reverse is still penalized:

```text
reverse_penalty: 1.10
change_penalty: 0.05
non_straight_penalty: 1.10
cost_penalty: 1.8
```

This means reverse is allowed when useful for tight warehouse maneuvers, but the
planner still prefers simple, smooth, lower-cost forward paths when available.

### Behavior Tree

The active Nav2 behavior tree computes and follows paths without spin or backup
recovery behaviors:

```text
ComputePathToPose planner_id=GridBased
FollowPath controller_id=FollowPath
Clear costmaps if planning or following fails
Wait as the only recovery behavior
```

Spin and generic backup recovery are intentionally removed because they can make
a forklift behave unrealistically in narrow aisles. The tree replans
periodically, clears stale obstacle data when needed, and then asks the
controller to continue following the latest path.

### Local Trajectory Follower

The visual navigation profile uses Nav2 MPPI as the local controller:

```text
controller_id: FollowPath
plugin: nav2_mppi_controller::MPPIController
motion_model: Ackermann
min_turning_r: 0.55
controller_frequency: 20 Hz
```

MPPI means Model Predictive Path Integral control. At every control tick, it
samples many short candidate control sequences, rolls them forward through the
motion model, scores the resulting trajectories, and sends the best immediate
velocity command.

In simplified form:

```text
for each control cycle:
    read fused pose from map -> odom -> base_footprint
    read global path and local costmap
    sample candidate velocity commands
    simulate candidate trajectories over a short horizon
    score each trajectory with critics
    publish the first command from the best trajectory
```

The important MPPI parameters in this repo are:

```text
time_steps: 28
model_dt: 0.05
batch_size: 1000
vx_max: 0.32
vx_min: -0.18
wz_max: 0.45
ax_max: 0.55
ax_min: -0.65
az_max: 0.70
```

So each control update evaluates about 1.4 seconds of possible future motion
using 1000 sampled trajectories. The speed and acceleration limits are kept low
because the simulated forklift is large relative to warehouse aisles and because
visual odometry benefits from smoother camera motion.

### MPPI Critics

MPPI chooses a trajectory by combining several cost terms, called critics:

```text
ConstraintCritic
CostCritic
GoalCritic
GoalAngleCritic
PathAlignCritic
PathFollowCritic
PathAngleCritic
PreferForwardCritic
```

Their roles are:

- `ConstraintCritic` penalizes trajectories that violate motion constraints.
- `CostCritic` penalizes trajectories close to obstacles in the costmap.
- `GoalCritic` pulls the robot toward the final goal position.
- `GoalAngleCritic` encourages the final heading near the goal to match.
- `PathAlignCritic` encourages the robot heading to align with the path.
- `PathFollowCritic` encourages staying close to the global path.
- `PathAngleCritic` penalizes bad angular approach to the path.
- `PreferForwardCritic` biases the solution toward forward driving when possible.

This is why the controller behaves better than a simple PID-to-path approach in
this setup. A PID follower mainly reacts to cross-track and heading error. MPPI
looks ahead, checks obstacle cost, respects acceleration and turning limits, and
can choose smoother commands before the forklift is already too far from the
path.

### Command Smoothing

The MPPI output is not sent straight to the Gazebo plugin. It first passes through
`planar_motion_guard.py`.

That node keeps the architecture practical for the current planar simulation:

```text
MPPI command
    -> low-speed turn smoothing / guard logic
    -> final /cmd_vel
```

It does not replace Nav2 planning or control. Its role is to reduce unstable
low-speed command behavior before the command reaches the planar Gazebo drive
plugin.

## Important Files

- `tools/run_visual_nav_manual.sh`
  Main command for manual testing and demo.

- `tests/standalone/`
  Lightweight algorithm tests that run without Gazebo, RViz, or Nav2.

- `src/warehouse_visual_localization/warehouse_visual_localization/core/`
  Shared algorithm modules imported by standalone tests and ROS nodes.

- `src/warehouse_visual_localization/config/core/forklift_2d.yaml`
  Common 2D vehicle, controller, friction, localization, and safety parameters.

- `src/warehouse_visual_localization/launch/nav_with_estimated_pose.launch.py`
  Top-level launch for the visual navigation pipeline.

- `src/warehouse_visual_localization/launch/visual_pose.launch.py`
  Starts simulated sensors, RGB-D odometry, EKF fusion, optional evaluator, and debug nodes.

- `src/warehouse_visual_localization/launch/sim_sensors.launch.py`
  Starts Gazebo, RViz, and the simulated forklift with RGB, depth, stereo, and lidar sensors.

- `src/warehouse_visual_localization/launch/rgbd_odom.launch.py`
  Starts RTAB-Map RGB-D odometry.

- `src/warehouse_visual_localization/config/ekf_visual.yaml`
  EKF configuration for fusing `/sim_wheel_odom` and `/visual_odom`.

- `src/warehouse_visual_localization/config/nav2_params_visual.yaml`
  Nav2 configuration used by this visual pipeline.

- `src/forklift_nav_bringup/launch/forklift_nav_stack.launch.py`
  Shared Nav2 bringup used by the visual pipeline.

- `src/forklift_nav_bringup/worlds/small_warehouse_open_top.world`
  Gazebo warehouse world.

- `src/forklift_nav_bringup/maps/warehouse_map.yaml`
  Static occupancy map loaded by Nav2 and RViz.

## Standalone Algorithm Test Environments

The repository also provides lightweight test environments for individual
algorithm groups. These do not start the warehouse scene, Gazebo GUI, RViz, or
the Nav2 stack.

Run all standalone tests:

```bash
cd /path/to/ros2_forklift_warehouse_ws
bash tests/standalone/run_all_standalone_tests.sh
```

Run one test:

```bash
python3 tests/standalone/kinematics_dynamics_demo.py
python3 tests/standalone/slam_localization_demo.py
python3 tests/standalone/slip_friction_demo.py
python3 tests/standalone/velocity_profile_demo.py
python3 tests/standalone/cbf_safety_demo.py
python3 tests/standalone/controller_tracking_demo.py
```

Each test writes logs, summary JSON, CSV, and plots to:

```text
$HOME/ros2_forklift_warehouse_artifacts/standalone_tests
```

The structure is intentionally split by responsibility:

```text
warehouse_visual_localization/core/
    shared formulas and algorithms

tests/standalone/
    independent Matplotlib-based validation environments

warehouse_visual_localization/ros_nodes/
    ROS adapter documentation; ROS nodes stay thin

warehouse_visual_localization/gazebo/
    Gazebo adapter documentation; no duplicated dynamics formulas
```

The important rule is that standalone tests must not copy formulas from ROS
nodes. ROS nodes and tests should import the same core modules, so changing a
vehicle parameter, safety formula, or controller rule in `core/` updates every
environment.

Current standalone groups:

- `kinematics_dynamics_demo.py`: 2D forklift body motion, steering angle,
  trajectory, center of mass proxy, yaw rate, and force estimates.
- `slam_localization_demo.py`: wheel-odometry prediction plus visual-odometry
  correction using a lightweight EKF.
- `slip_friction_demo.py`: tire slip ratio, available friction force, and force
  saturation.
- `velocity_profile_demo.py`: velocity and acceleration profile over distance.
- `cbf_safety_demo.py`: CBF-style speed/yaw-rate filtering near obstacles.
- `controller_tracking_demo.py`: trajectory tracking on a synthetic reference
  path using the shared vehicle model.

### Optional Canonical Constraints Framework

The repository also contains an opt-in, pure-Python framework linking path
curvature, footprint corridor margins, quasi-static rollover screening, human
separation, docking geometry, visual-servo micro-control, and linear CBF
filtering. It does not change the default Nav2, MPPI, RTAB-Map, EKF, Gazebo, or
`planar_motion_guard.py` behaviour.

Run its headless scenarios with:

```bash
cd "$(git rev-parse --show-toplevel)"
bash tests/standalone/run_canonical_constraints_tests.sh
```

See [canonical equations](docs/canonical_equations.md) for formulas, units,
limits, and the optional disabled-by-default ROS monitor.

Run optional integration checks:

```bash
bash tests/integration/run_canonical_integration_tests.sh
```

Run the opt-in Gazebo monitor scenario without changing the default demo:

```bash
HEADLESS=1 bash tests/gazebo/run_curvature_speed_scenario.sh
```

The Gazebo integration is planar: rollover is an algorithmic ZMP/utilization
monitor only, not physical chassis rollover simulation. The benchmark runner
first writes a deterministic core preflight; runtime claims require a completed
Gazebo goal and recorded ROS-topic artifacts.

## Interactive Algorithm Labs

For manual algorithm inspection, use the interactive Matplotlib labs. Each lab
has its own folder under `tests/interactive/labs/<lab_name>/` with `app.py`,
`README.md`, and room for future `controllers/`, `scenarios/`, or `assets/`.
Backward-compatible wrappers remain at `tests/interactive/*_lab.py`.

Run through wrappers:

```bash
python3 tests/interactive/kinematics_dynamics_lab.py
python3 tests/interactive/cbf_safety_lab.py
python3 tests/interactive/controller_tracking_lab.py
python3 tests/interactive/slip_friction_lab.py
python3 tests/interactive/velocity_profile_lab.py
python3 tests/interactive/localization_lab.py
```

These windows have right-side sliders and buttons. The kinematics lab is
wheel-level: it exposes individual `FL/FR/RL/RR` wheel speeds, rear steering,
`dt`, and number of integration steps, then solves the body twist from the four
wheel constraints. The CBF lab follows an octagonal safe-set animation style: red
area is unsafe, the center of mass moves toward a clicked reference point, and
CBF projects the desired velocity back into the safe set. Use `Play/Pause` for
continuous animation or `Step` for frame-by-frame inspection.

The controller lab uses a dropdown-style selector to switch between Pure
Pursuit, Stanley, and MPPI. Its right panel is grouped with
separators for controller choice, common parameters, one dynamic
algorithm-parameter section, and actions. MPPI samples noisy short-horizon
control sequences, rolls them out through the shared forklift model, scores them,
and uses softmin weighting to update the nominal control sequence.

Headless validation:

```bash
bash tests/interactive/run_interactive_smoke.sh
```

## Main Topics

Camera topics:

```text
/rgb_camera/image_raw
/depth_camera/image_raw
/depth_camera/depth/image_raw
/depth_camera/camera_info
/stereo_left_camera/image_raw
/stereo_right_camera/image_raw
```

Odometry and TF:

```text
/sim_wheel_odom
/visual_odom
/odom
map -> odom -> base_footprint
```

Navigation:

```text
/goal_pose
/plan
/visual_nav/cmd_vel_request
/cmd_vel
```

## Requirements

Target environment:

- Ubuntu 22.04 under WSL
- ROS 2 Humble
- Gazebo Classic 11
- Nav2
- RTAB-Map ROS for Humble
- robot_localization

The helper scripts infer the workspace from their own location. The repository
can be cloned into any directory, for example:

```text
$HOME/ros2_forklift_warehouse_ws
```

Build artifacts are written outside the source tree by default:

```text
$HOME/ros2_forklift_warehouse_artifacts
```

Override this location with `ARTIFACT_ROOT` if needed.

## Setup

Clone with submodules:

```bash
git clone --recurse-submodules git@github.com:TueNguyen2006/ros2_forklift_warehouse_ws.git
cd ros2_forklift_warehouse_ws
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Install/bootstrap dependencies:

```bash
bash tools/bootstrap_ros2_humble.sh
```

Build:

```bash
bash tools/build_workspace.sh
```

## Run

From inside WSL:

```bash
cd /path/to/ros2_forklift_warehouse_ws
bash tools/run_visual_nav_manual.sh
```

If Gazebo or RViz is already running from a previous test:

```bash
cd /path/to/ros2_forklift_warehouse_ws
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
bash tools/run_visual_nav_manual.sh
```

The script starts:

- Gazebo warehouse world
- simulated forklift
- RGB camera
- depth camera
- stereo cameras
- RTAB-Map RGB-D odometry
- EKF odometry fusion
- Nav2
- RViz visualization
- Gazebo goal bridge

## Manual Test Flow

1. Start the pipeline with `bash tools/run_visual_nav_manual.sh`.
2. Wait until Gazebo and RViz are fully loaded.
3. In Gazebo, click `Set Nav Goal`.
4. Click a reachable floor location in the warehouse.
5. Watch RViz for the planned path.
6. Watch Gazebo for the forklift motion.
7. Watch RViz camera panels for RGB, depth, and stereo images.

Healthy runtime signs:

```text
Pose source=rgbd_odom_fused
state=ready
source_topics=ok
map->odom=ok
odom->base=ok
```

## Localization Mode

The default manual mode uses:

```text
localization:=false
```

In this mode the static warehouse map is always loaded for Nav2 planning, and a
static `map -> odom` transform is used for manual testing. The robot motion pose
used by Nav2 still comes from the EKF output built from `/sim_wheel_odom` and
`/visual_odom`.

RTAB-Map global localization mode exists as a later extension, but it is not the
default stable demo path.

## What This Repository Does Not Focus On Now

The current repository focus is not physical rear-steer forklift tire dynamics.
That work was intentionally removed from the main path so the project has one
clean target:

```text
camera + wheel odometry -> EKF pose -> Nav2 warehouse navigation
```

## WSLg Notes

If the environment script reports:

```text
/mnt/shared_memory is missing
```

Gazebo or RViz may show blank windows or WSLg copy mode artifacts. From Windows
PowerShell, restart WSL:

```powershell
wsl --shutdown
```

Then reopen Ubuntu 22.04 and run the launch script again.
