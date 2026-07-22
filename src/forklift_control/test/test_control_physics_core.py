import unittest

from forklift_control.actuator_limits import ActuatorCommand, ActuatorLimitConfig, limit_command
from forklift_control.kinematics import ForkliftKinematicConfig, integrate_odometry, yaw_rate_from_state


class ControlPhysicsCoreTest(unittest.TestCase):
    def test_control_core_limits_and_integrates_motion(self):
        cfg = ForkliftKinematicConfig(steering_axle="rear")
        state = integrate_odometry((0.0, 0.0, 0.0), 1.0 / cfg.wheel_radius, 0.0, 0.1, cfg)
        self.assertGreater(state[0], 0.09)
        self.assertLess(abs(state[1]), 1e-6)
        self.assertLess(abs(state[2]), 1e-6)

        self.assertGreater(yaw_rate_from_state(0.5, -0.2, cfg), 0.0)

        limited = limit_command(
            ActuatorCommand(speed=0.0, steering_angle=0.0),
            ActuatorCommand(speed=1.0, steering_angle=0.55),
            dt=0.1,
            config=ActuatorLimitConfig(max_accel=0.5, max_steering_rate=0.5),
        )
        self.assertEqual(limited.speed, 0.05)
        self.assertEqual(limited.steering_angle, 0.05)
