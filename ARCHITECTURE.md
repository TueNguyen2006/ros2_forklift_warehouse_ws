# Forklift Warehouse ROS 2 Architecture

This workspace is organized around stable package boundaries while preserving the existing visual navigation behavior.

## Package Responsibilities

- `forklift_description`: forklift URDF/Xacro, robot description config, RViz description views, and future meshes.
- `forklift_simulation`: Gazebo Classic warehouse worlds, Gazebo model assets, map generation helpers, and simulator launch entry points.
- `forklift_navigation`: Nav2 config, behavior trees, occupancy maps, navigation launch, initial pose helper, and navigation debug logging.
- `forklift_safety`: runtime safety nodes such as stability guarding and synthetic odometry-to-IMU conversion.
- `warehouse_visual_localization`: visual odometry/localization pipeline, RTAB-Map launch, EKF fusion, visual navigation adapters, and current simulator-independent research core.
- `forklift_evaluation`: route matrix execution, scenario files, metrics, and benchmark launch entry points.
- `forklift_operator_tools`: manual operator tools such as the Gazebo Classic click-to-Nav2-goal GUI plugin.
- `forklift_bringup`: integrated launch files and run-mode wiring only.
- `third_party`: upstream packages and external assets. Do not modify upstream source for project-specific behavior.

## Transitional Boundaries

`warehouse_visual_localization.core` still contains controller, vehicle model, friction, localization, and safety formulas used by current tests and ROS adapters. These modules should move to `forklift_model`, `forklift_control`, or `forklift_safety` only after a focused migration preserves imports, tests, and behavior.

`forklift_simulation/launch/simulation.launch.py` currently wraps the existing visual simulator launch to preserve the active planar Gazebo model. A future extraction should move the shared simulator description helpers out of `warehouse_visual_localization`.

## Launch Layers

- Low-level simulation: `ros2 launch forklift_simulation simulation.launch.py`
- Low-level navigation: `ros2 launch forklift_navigation navigation.launch.py`
- Low-level safety: `ros2 launch forklift_safety safety.launch.py`
- Integrated visual stack: `ros2 launch forklift_bringup bringup.launch.py`
- Evaluation: `ros2 launch forklift_evaluation evaluation.launch.py`
