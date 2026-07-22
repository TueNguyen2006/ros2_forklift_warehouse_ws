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

The model intentionally does not use `diff_drive_controller`. `/cmd_vel` is converted through an Ackermann/rear-steer adapter into target speed and steering angle, then into steering and drive joint commands.

Values above are starter values for simulation bringup. They must be replaced with measurements from the real forklift before using the model for sim-to-real claims.
