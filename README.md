# ROS 2 Forklift Warehouse Navigation Workspace

This workspace implements a simulation-first forklift navigation stack for `WSL Ubuntu-22.04` using `ROS 2 Humble`, `Nav2`, and `Gazebo Classic 11`.

## Workspace Layout

- `src/third_party/ROS2-Forklift-Simulation`
  Forklift upstream repository kept intact for the baseline vehicle model and custom Gazebo collision plugin.
- `src/third_party/aws-robomaker-small-warehouse-world`
  Warehouse upstream repository kept intact as the original source of world, map, and model assets.
- `src/forklift_nav_bringup`
  Baseline and realistic simulation launch files, Nav2 configs, behavior trees, maps, and costmap filter masks.
- `src/forklift_safety`
  `stability_guard`, synthetic IMU node for the baseline model, and the route-matrix runner.
- `src/forklift_description_realistic`
  Rear-steer forklift description using `ackermann_steering_controller`.
- `tools`
  Bootstrap, build, launch, and route-test helper scripts for WSL.

## Recommended Environment

- Ubuntu `22.04`
- ROS 2 `Humble`
- Nav2 `Humble`
- Gazebo Classic `11`
- `colcon`, `rosdep`, `vcstool`

## Quick Start

1. Install ROS 2 / Nav2 / Gazebo dependencies:

   ```bash
   cd /home/tuenguyen/ros2_forklift_warehouse_ws
   ./tools/bootstrap_ros2_humble.sh
   ```

2. Build the workspace:

   ```bash
   ./tools/build_workspace.sh
   ```

   Build, install, and log artifacts are written to `~/ros2_forklift_warehouse_artifacts` by default so the low-space Windows drive is not used for generated files.

3. Run the baseline demo:

   ```bash
   ./tools/run_baseline_nav.sh
   ```

4. Run the rear-steer realistic model:

   ```bash
   ./tools/run_realistic_nav.sh
   ```

5. Run the route matrix:

   ```bash
   ./tools/run_route_matrix.sh
   ```

## Notes

- The `aws-robomaker-small-warehouse-world` package is not built as a ROS 2 package. Its assets are vendored into `forklift_nav_bringup` for runtime use.
- The `forklift_gym_env` package from the upstream forklift repository is intentionally skipped during build.
- The upstream forklift repository does not declare a license in `package.xml` / `setup.py`. Treat it as an internal prototype dependency until its redistribution status is clarified.
