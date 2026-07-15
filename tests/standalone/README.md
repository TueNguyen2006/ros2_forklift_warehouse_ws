# Standalone Algorithm Tests

These tests run without Gazebo, RViz, Nav2, or the warehouse scene. They import
the same core modules used by ROS/Gazebo integration.

Run all:

```bash
cd /path/to/ros2_forklift_warehouse_ws
bash tests/standalone/run_all_standalone_tests.sh
```

Run one:

```bash
python3 tests/standalone/kinematics_dynamics_demo.py
python3 tests/standalone/slam_localization_demo.py
python3 tests/standalone/slip_friction_demo.py
python3 tests/standalone/velocity_profile_demo.py
python3 tests/standalone/cbf_safety_demo.py
python3 tests/standalone/controller_tracking_demo.py
```

Add `--show` to open Matplotlib windows:

```bash
python3 tests/standalone/controller_tracking_demo.py --show
```

Outputs are written to:

```text
$HOME/ros2_forklift_warehouse_artifacts/standalone_tests
```

