# Canonical Equations Linking Planning, Control, Manipulation, and Safety

This document describes an optional, diagnostic-only framework in
`warehouse_visual_localization.core`. It is deliberately separate from the
default Nav2, MPPI, RTAB-Map, EKF, Gazebo, and `planar_motion_guard.py`
runtime. Default behaviour of the warehouse demo does not change.

All values in the YAML are conservative demo defaults, not certified physical
limits for a particular forklift, mast, tire, payload, or worksite. Validate
them with the actual vehicle specification and a safety process before use on a
real machine.

## 1. Curvature to Speed

For signed path curvature `kappa` (1/m), the reference speed limit is:

```text
v_max = min(v_bar, sqrt(a_y_max / (abs(kappa) + epsilon)))
```

`v_bar` is the speed cap (m/s), `a_y_max` is allowed lateral acceleration
(m/s^2), and `epsilon > 0` prevents division by zero. Curvature is estimated
from three consecutive geometric path points. Duplicate points and short paths
return zero curvature, not NaN. The optional smoothing only smooths the output
curvature; it never edits a Nav2 path.

If the quasi-static rollover model yields a lower lateral acceleration limit,
the effective value is `min(a_y_configured, a_y_roll)`. This module only
publishes a separate reference limit through the optional monitor; it does not
modify Nav2 MPPI.

## 2. Footprint in Corridor

Each footprint vertex is transformed from body to world coordinates:

```text
p_i_world = [x, y]^T + R(yaw) p_i_body
R(yaw) = [[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]]
```

The two linear boundaries use `n_L^T p <= b_L` and `n_R^T p >= b_R`. The result
contains every transformed vertex, per-corner violations, maximum violation,
minimum signed margin, and feasibility. The polygon may contain the body,
forks, payload, or their combined outline. Its origin is supplied in YAML and
is not assumed to be at the geometric center.

## 3. Lateral Acceleration and Quasi-static Rollover

```text
a_y = v^2 kappa
p_zmp = p_cog + h/g [a_x, a_y]^T
abs(a_y) <= T g cos(theta) / (2 h)
```

`T` is track width (m), `h` is combined CoG height (m), and `theta` is floor
grade (rad). The combined planar CoG is:

```text
p_cog_total = (m_vehicle p_vehicle + m_load p_load) / (m_vehicle + m_load)
```

The ZMP is checked in a supplied convex support polygon with an in-core
point-in-convex-polygon routine. This is a quasi-static screening model. It is
not a complete counterbalance forklift rollover model: it excludes tire
deformation, suspension, mast compliance, rear-axle pivot dynamics, load shift,
and transient roll.

## 4. Human Speed and Separation Monitoring

For robot speed `v`, braking capability `a_m`, and delay `T=t_r+t_c`:

```text
v^2 / (2 a_m) + v T <= z - z_0
v_max_ssm = -a_m T + sqrt((a_m T)^2 + 2 a_m (z-z_0))
```

The implementation returns delay distance, braking distance, stopping distance,
allowed speed, and one of `clear`, `speed_limited`, `stop`, or `invalid`.
Invalid braking capability and separation at/below the margin fail safe to zero
allowed speed. An optional approaching-human speed consumes separation during
the delay. No person detector is introduced by this framework.

## 5. Docking Geometry

At an insertion length `l`, lateral vehicle error `e_y`, and yaw error
`e_psi` in radians:

```text
e_tip_approx = e_y + l e_psi
e_tip_exact = e_y + l sin(e_psi)
abs(e_tip) <= w_p/2 - margin
abs(e_psi) <= (w_p/2 - margin) / l
```

The exact planar result is used for feasibility. The small-angle form is kept
as an explanatory diagnostic. This does not model mast deflection, fork taper,
pocket depth, pallet deformation, or perception calibration error.

## 6. Visual-servo Micro-control

IBVS receives image features and an interaction matrix:

```text
s_dot = L_s v_c
v_c = -lambda pinv(L_s) (s-s_star)
```

The output includes command, residual, matrix rank, condition number, and
validity. Shape, finite values, rank deficiency, and output limits are checked.

The PBVS adapter is planar only, suitable for forklift micro-docking:

```text
e = [e_x, e_y, wrap(e_psi)]^T
u = -K e
```

It is not a full SE(3) logarithm implementation.

## 7. Linear CBF Command Projection

The generic continuous and discrete CBF ideas are:

```text
grad(h)^T (f(x) + g(x)u) + alpha h(x) >= 0
h(x_next) - (1-eta) h(x) >= 0
```

The initial implementation intentionally only solves the linear-control-space
case `A u <= b` with actuator bounds. It minimizes the distance to the desired
command using repeated half-space projection and box clamping. It is not a
general nonlinear QP solver. Invalid input, contradictory constraints, or
non-convergence return a bounded zero command as fail-safe fallback.

## Unified Evaluator and ROS Adapter

`CanonicalConstraintEvaluator` combines any available groups into a report.
Missing inputs stay `unavailable`; no synthetic sensor data is invented.

The optional monitor is disabled twice by default: ROS parameter `enabled` is
false and `canonical_constraints.enabled` in YAML is false. When explicitly
enabled, it reads `/plan`, `/odom`, and optionally
`/forklift/canonical_constraints/human_distance`, then publishes only:

```text
/forklift/canonical_constraints/status
/forklift/reference_speed_limit
```

It never publishes `/cmd_vel` and is not part of any default launch file.

## Run

```bash
cd "$(git rev-parse --show-toplevel)"
bash tests/standalone/run_canonical_constraints_tests.sh
python3 -m pytest -q tests/unit
```

Artifacts are written under:

```text
$HOME/ros2_forklift_warehouse_artifacts/standalone_tests/canonical_constraints
```
