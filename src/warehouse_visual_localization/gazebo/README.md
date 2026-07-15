# Gazebo Integration

Gazebo-specific launch files, plugins, and world integration should use the same
core algorithms as standalone tests and ROS nodes.

For the current stable branch, Gazebo provides:

```text
warehouse scene
forklift body and sensors
planar motion plugin
simulated wheel odometry
RGB-D and stereo camera streams
```

Do not duplicate vehicle dynamics formulas here. Add reusable math to
`warehouse_visual_localization.core` first, then call it from Gazebo adapters.

