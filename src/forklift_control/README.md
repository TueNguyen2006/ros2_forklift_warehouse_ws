# forklift_control

Planned home for reusable forklift controllers and ROS 2 control configuration.

Current controller algorithms remain in `warehouse_visual_localization.core.controller` to preserve behavior while the package boundaries are introduced. Future moves should keep topic names, parameter names, and controller outputs compatible with the existing Nav2 and Gazebo launch files.

Expected future layout:

```text
steering_controller/
lift_controller/
mpc/
ros2_control/
```
