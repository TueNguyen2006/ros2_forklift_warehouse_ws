# Camera-Based Warehouse Navigation For Forklift

This repository has one main purpose:

Run a forklift in a Gazebo warehouse using simulated camera and wheel-odometry
sensors, then feed the estimated pose to Nav2 for path planning and path
following.

The main entry point is:

```bash
bash tools/run_visual_nav_manual.sh
```

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

## Important Files

- `tools/run_visual_nav_manual.sh`
  Main command for manual testing and demo.

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
