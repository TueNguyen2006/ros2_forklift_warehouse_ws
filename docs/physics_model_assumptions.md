# Physics Model Assumptions

The current physics mode is an initial engineering model, not a calibrated digital twin.

Default assumptions:

- steering axle: rear;
- drive axle: front;
- wheelbase: `1.35 m`;
- track width: `1.05 m`;
- wheel radius: `0.16 m`;
- max steering angle: `0.55 rad`;
- max steering rate: `0.65 rad/s`;
- max forward speed: `0.75 m/s`;
- max reverse speed: `0.35 m/s`.
- default physics spawn z: `0.30 m`, so the physics model starts above the floor and Gazebo lets the wheels settle into contact instead of spawning directly inside the contact surface.
- physics frame convention: `base_link` is the Gazebo root/inertial link at wheel-center height; `base_footprint` is a fixed child frame projected below it for ROS compatibility.
- default physics world: `physics_floor.world`, a simple Gazebo plane named `normal_floor`.

The physics default intentionally does not use the AWS warehouse ground mesh. That mesh is kept for planar navigation, but its collision surface currently uses very high friction (`mu=100`, `mu2=50`) with zero slip. That is a poor first contact target for wheel-joint debugging because it can make the wheels appear to stick, sink, or drift laterally. Use the simple floor first to validate joints, odometry, and controller behavior, then reintroduce warehouse visuals/collisions as a separate calibration step.

The model intentionally does not use `diff_drive_controller`. `/cmd_vel` is converted through an Ackermann/rear-steer adapter into target speed and steering angle, then into steering and drive joint commands.

Values above are starter values for simulation bringup. They must be replaced with measurements from the real forklift before using the model for sim-to-real claims.
