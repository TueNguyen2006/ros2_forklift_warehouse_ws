# ROS Nodes

ROS nodes should stay thin. They translate ROS messages, parameters, and TF into
calls to `warehouse_visual_localization.core`.

Current example:

```text
scripts/planar_motion_guard.py
    -> imports core.command_shaper.CommandShaper
```

Rule:

```text
ROS node = integration layer
core module = algorithm and formulas
```

