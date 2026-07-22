# Migration Map

Files were copied first so the legacy locations can remain available until build and smoke checks pass.

| Old location | New location | Notes |
| --- | --- | --- |
| `src/forklift_nav_bringup/worlds/` | `src/forklift_simulation/worlds/` | Gazebo Classic warehouse worlds. |
| `src/forklift_nav_bringup/models/` | `src/forklift_simulation/models/` | Gazebo model assets used by warehouse worlds. |
| `src/forklift_nav_bringup/forklift_nav_bringup/world_map_generator.py` | `src/forklift_simulation/forklift_simulation/world_map_generator.py` | Runtime map generation helper. |
| `src/forklift_nav_bringup/launch/warehouse_nav_baseline.launch.py` | `src/forklift_simulation/launch/warehouse_nav_baseline_legacy_helpers.launch.py` | Transitional helper source for building the current planar robot description; not a new public entrypoint. |
| `src/forklift_nav_bringup/config/` | `src/forklift_navigation/config/` | Nav2 and collision monitor config. |
| `src/forklift_nav_bringup/maps/` | `src/forklift_navigation/maps/` | Occupancy maps and filter masks. |
| `src/forklift_nav_bringup/behavior_trees/` | `src/forklift_navigation/behavior_trees/` | Nav2 behavior trees. |
| `src/forklift_nav_bringup/rviz/` | `src/forklift_navigation/rviz/` | Navigation RViz config. |
| `src/forklift_nav_bringup/launch/forklift_nav_stack.launch.py` | `src/forklift_navigation/launch/navigation.launch.py` | Nav2-only launch entry. |
| `src/forklift_nav_bringup/forklift_nav_bringup/initial_pose_publisher.py` | `src/forklift_navigation/forklift_navigation/initial_pose_publisher.py` | Navigation helper node. |
| `src/forklift_nav_bringup/forklift_nav_bringup/nav_debug_logger.py` | `src/forklift_navigation/forklift_navigation/nav_debug_logger.py` | Navigation debug helper. |
| `src/third_party/ROS2-Forklift-Simulation/src/forklift_robot/urdf/` | `src/forklift_description/urdf/` | Project-owned copy of robot description assets. |
| `src/third_party/ROS2-Forklift-Simulation/src/forklift_robot/config/` | `src/forklift_description/config/` | Description/controller config copy. |
| `src/third_party/ROS2-Forklift-Simulation/src/forklift_robot/rviz/` | `src/forklift_description/rviz/` | Description RViz config copy. |
| `src/forklift_safety/scenarios/` | `src/forklift_evaluation/scenarios/` | Route matrix scenarios copied for evaluation ownership. |
| `src/forklift_safety/forklift_safety/nav_matrix_runner.py` | `src/forklift_evaluation/forklift_evaluation/nav_matrix_runner.py` | Evaluation runner copied; legacy entry remains until final cleanup. |
| `src/gazebo_nav_goal_tool/src/` | `src/forklift_operator_tools/src/` | Operator GUI plugin copied; legacy package remains until final cleanup. |

## Legacy Paths Kept

`forklift_nav_bringup`, `forklift_safety/scenarios`, and `gazebo_nav_goal_tool` are not deleted in this migration pass. They should be removed or marked `COLCON_IGNORE` only after full Gazebo, Nav2 lifecycle, TF, and route smoke tests pass.
