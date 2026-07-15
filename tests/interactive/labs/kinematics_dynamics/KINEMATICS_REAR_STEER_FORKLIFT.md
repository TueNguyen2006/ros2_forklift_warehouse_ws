# Rear-Steer 4-Wheel Forklift Kinematics

This note documents the kinematics used by
`tests/interactive/kinematics_dynamics_lab.py`.

The model is intentionally kinematic, not tire-dynamic. It answers:

```text
Given four wheel rolling speeds and rear steering angle,
what planar body velocity is consistent with a rear-steer forklift?
```

## Coordinate Frame

Body frame:

```text
x: forward
y: left
yaw: counter-clockwise
```

For rear-steer forklift kinematics, the reference point used by this test is the
midpoint of the **fixed front axle**.

The front-axle reference twist is:

```text
q_dot = [vx_front, vy_front, omega]
```

Because the front wheels are not steered, the no-side-slip constraint is applied
at the front axle:

```text
vy_front = 0
```

The vehicle center of mass is displayed separately in the interactive lab. During
turning, the CG can have a lateral velocity even though the front axle reference
has `vy_front = 0`.

## Wheel Positions

The origin is the midpoint of the front axle.

```text
front_left  = (0, +W/2)
front_right = (0, -W/2)
rear_left   = (-L, +W/2)
rear_right  = (-L, -W/2)
```

Where:

```text
L = wheelbase
W = track width
```

## Velocity At Each Wheel

For a rigid body moving in the plane, the velocity at a wheel located at
`r_i = (x_i, y_i)` is:

```text
v_i = [vx_front, vy_front] + omega x r_i
```

In 2D:

```text
v_ix = vx_front - omega * y_i
v_iy = vy_front + omega * x_i
```

Because the fixed front axle is nonholonomic:

```text
vy_front = 0
```

So:

```text
v_ix = vx_front - omega * y_i
v_iy = omega * x_i
```

## Wheel Rolling Constraint

Each wheel has a rolling direction:

```text
d_i = [cos(delta_i), sin(delta_i)]
```

The measured or commanded rolling speed of that wheel is:

```text
s_i = d_i dot v_i
```

Expanded:

```text
s_i = cos(delta_i) * (vx - omega * y_i)
    + sin(delta_i) * (omega * x_i)
```

Therefore:

```text
s_i = cos(delta_i) * vx_front
    + (-cos(delta_i) * y_i + sin(delta_i) * x_i) * omega
```

## Rear-Steer Forklift Assumption

For a rear-steer forklift:

```text
front wheel steering angle = 0
rear wheel steering angle  = delta_r
```

So:

```text
delta_front_left  = 0
delta_front_right = 0
delta_rear_left   = delta_r
delta_rear_right  = delta_r
```

The four wheel equations become an overdetermined least-squares problem:

```text
A * [vx_front, omega]^T = s
```

Where each row is:

```text
[cos(delta_i), -cos(delta_i) * y_i + sin(delta_i) * x_i]
```

And:

```text
s = [s_FL, s_FR, s_RL, s_RR]^T
```

The solver returns:

```text
vx_front
omega
residual = ||A * [vx_front, omega]^T - s||
```

## Meaning Of Residual

The residual is important.

If all wheel speeds and steering angles are physically consistent:

```text
residual ~= 0
```

If the user manually sets wheel speeds that contradict the rigid-body rear-steer
model:

```text
residual > 0
```

Example contradiction:

```text
front_left speed != front_right speed
rear_left speed == rear_right speed
rear steering angle != 0
```

This can imply a mix of turning, side slip, wheel slip, or an impossible rigid
body motion. The interactive lab marks this through the residual number.

## Ideal Rear-Steer Bicycle Approximation

For an ideal rear-steer bicycle model:

```text
omega = -vx_front * tan(delta_r) / L
```

The negative sign appears because steering the rear axle left rotates the vehicle
opposite the front-steer convention.

The interactive lab's `Ideal` button uses this relationship to generate
consistent wheel speeds for the selected `ideal vx` and `rear steer`.

## Why The Previous Lab Looked Wrong

The first version solved:

```text
[vx, vy, omega]
```

from four wheel constraints.

That is closer to an omni/slipping body model because it allowed lateral body
velocity at the body reference. A real rear-steer forklift should apply the
nonholonomic no-side-slip constraint at the fixed front axle, so the current lab
constrains:

```text
vy_front = 0
```

and solves only:

```text
[vx_front, omega]
```
