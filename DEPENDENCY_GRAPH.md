# Dependency Graph

Preferred dependency direction:

```text
forklift_interfaces, forklift_description
    -> forklift_simulation, forklift_control, forklift_localization, forklift_perception, forklift_safety
    -> forklift_navigation, forklift_evaluation, forklift_rl
    -> forklift_bringup
```

Current package dependency intent:

```text
forklift_description
    -> forklift_simulation

forklift_safety
    -> forklift_navigation

forklift_description
forklift_simulation
forklift_navigation
    -> warehouse_visual_localization

forklift_navigation
forklift_safety
warehouse_visual_localization
forklift_simulation
    -> forklift_bringup

forklift_evaluation
    -> runtime Nav2 stack

forklift_operator_tools
    -> Gazebo Classic / rclcpp / geometry_msgs
```

`third_party` packages are external inputs and should not depend on project-owned packages.

Known transitional edge: `forklift_simulation/launch/simulation.launch.py` wraps `warehouse_visual_localization/launch/sim_sensors.launch.py` to preserve the current planar visual simulator behavior without rewriting launch internals.
