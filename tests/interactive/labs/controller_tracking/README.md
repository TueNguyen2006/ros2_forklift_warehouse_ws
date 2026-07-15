# Controller Tracking Lab

Purpose:

```text
Compare path-tracking controllers on the same forklift model and reference path.
```

Current controllers:

```text
Pure Pursuit
Stanley
MPPI
SBMPC-JAX
```

Current scenarios:

```text
Sine Offset
Circle On Path
U-Turn Start Line
Figure Eight On Path
Warehouse Chicane
Tight Hairpin Stress
```

`Circle On Path` and `U-Turn Start Line` spawn the forklift directly on the
reference line with matching yaw. These scenarios are intended to isolate
controller quality from initial-pose recovery.

Scenario intent:

```text
Circle On Path
    Closed-loop path. Verifies controllers do not stop immediately when
    path[-1] is close to path[0].

U-Turn Start Line
    Feasible 180-degree turn. Verifies rear-steer tracking from an exact
    start-line pose.

Figure Eight On Path
    Hard feasible crossing path. Tests near-limit curvature plus path
    ambiguity at the center crossing.

Warehouse Chicane
    Hard feasible aisle-like S-turn. Tests repeated steering reversals near
    the forklift steering limit.

Tight Hairpin Stress
    Intentionally infeasible. Required steering is about 0.785 rad while the
    model limit is 0.62 rad. Large error here is expected and marks the
    physical limit, not automatically a controller bug.
```

MPPI backend:

```text
warehouse_visual_localization._mppi_native
```

The MPPI rollout/cost/softmin loop is implemented as a C++ Python extension
and loaded by `MPPIController` when the package has been built. If the native
extension is not available, the controller falls back to the pure Python MPPI
implementation so the lab still runs.

Expected title label:

```text
MPPI/cpp
```

If the title shows `MPPI/python`, rebuild the package:

```bash
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select warehouse_visual_localization
source install/setup.bash
python3 tests/interactive/controller_tracking_lab.py
```

SB-MPC / Feedback-MPPI reference:

```text
Source repo: https://github.com/tombelv/sbmpc
Inspected commit: f5bd3a0969ba1eba956cb9588b024c90a5e9fbd0
```

The lab implements a lightweight JAX sampling-based MPC backend inspired by
`tombelv/sbmpc` instead of vendoring the package directly. Reason: the upstream
package currently declares `jax>=0.8.0` and MuJoCo/MJX dependencies, while this
Ubuntu 22.04 / Python 3.10 environment resolves up to JAX 0.6.x from pip index.
For this 2D forklift controller lab we only need the core SB-MPC pattern:

```text
nominal control sequence
    -> sample noisy candidate sequences
    -> vectorized rollout with forklift model
    -> path/heading/control cost
    -> softmin weighted control update
    -> shift nominal sequence for the next control tick
```

The implementation lives in:

```text
warehouse_visual_localization.core.controller.SBMPCJAXController
```

Dependency installed for this lab:

```bash
python3 -m pip install --user "jax[cpu]==0.4.35" "numpy==1.26.4"
```

Measured on this WSL CPU environment after warm-start guidance was added:

```text
MPPI C++: 4-6 ms/control at horizon 36, batch 320
SBMPC-JAX: 13-16 ms/control at horizon 36, batch 320
```

Conclusion: JAX SB-MPC is fast enough for interactive controller testing after
warm-up. MPPI C++ remains the lower-latency option for tight real-time loops.

Controller benchmark:

```bash
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source install/setup.bash
python3 tests/interactive/labs/controller_tracking/benchmark.py
```

Latest benchmark summary:

```text
Sine Offset: PP/MPPI/SBMPC tail RMS about 3-4 cm; Stanley about 8 cm.
Circle On Path: PP/MPPI/SBMPC tail RMS about 1.6-1.9 cm; Stanley about 4.2 cm.
U-Turn Start Line: PP/MPPI/SBMPC tail RMS about 1.8-2.4 cm; Stanley about 4.0 cm.
Figure Eight On Path: all controllers tail RMS about 6.5 cm; this is the
    hardest feasible case because the path is near the steering limit and
    crosses itself.
Warehouse Chicane: all controllers tail RMS about 1.3-1.8 cm.
Tight Hairpin Stress: all controllers exceed 22 cm tail RMS because the path
    asks for more steering than the model can physically provide.
```

Important controller bug fixed:

```text
Closed-loop paths must not trigger terminal-goal stopping.
```

Before this fix, a circular path could look perfect in metrics because the
vehicle never moved: `path[-1]` was close to `path[0]`, so goal tolerance
returned zero command immediately.

Structure:

```text
app.py
    Matplotlib UI, dropdown controller/scenario selector, controller-specific parameter panel.

scenarios.py
    Shared scenario generation for UI and benchmark.

benchmark.py
    Reproducible controller benchmark over all scenarios.

assets/
    Future exported plots or animations.
```

Controller formulas live in `warehouse_visual_localization.core.controller`.
The lab should import controllers from core instead of duplicating algorithms.
