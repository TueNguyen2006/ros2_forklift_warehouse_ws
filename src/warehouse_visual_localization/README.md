# warehouse_visual_localization

This package contains the visual navigation pipeline used by the workspace.

Main entry point from the workspace root:

```bash
bash tools/run_visual_nav_manual.sh
```

Pipeline:

```text
RGB-D camera -> RTAB-Map RGB-D odometry -> /visual_odom
/sim_wheel_odom + /visual_odom -> robot_localization EKF -> /odom
Nav2 consumes map -> odom -> base_footprint
```

The full project documentation is in the workspace root README:

```text
README.md
```
