import yaml

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from forklift_control.actuator_limits import (
    ActuatorCommand,
    ActuatorLimitConfig,
    limit_command,
)
from forklift_control.kinematics import (
    ForkliftKinematicConfig,
    steering_angle_from_twist,
    wheel_angular_velocity,
)


def _load_yaml(path: str) -> dict:
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class AckermannCommandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("ackermann_command_adapter")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("target_command_topic", "/forklift/control/target")
        self.declare_parameter("steering_command_topic", "/steering_position_controller/commands")
        self.declare_parameter("drive_command_topic", "/drive_velocity_controller/commands")
        self.declare_parameter("config_path", "")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("wheelbase", 1.35)
        self.declare_parameter("wheel_radius", 0.16)
        self.declare_parameter("steering_axle", "rear")
        self.declare_parameter("drive_axle", "front")

        cfg = _load_yaml(str(self.get_parameter("config_path").value))
        kin_cfg = cfg.get("kinematics", {})
        limits_cfg = cfg.get("actuator_limits", {})
        self.kinematics = ForkliftKinematicConfig(
            wheelbase=float(kin_cfg.get("wheelbase", self.get_parameter("wheelbase").value)),
            wheel_radius=float(kin_cfg.get("wheel_radius", self.get_parameter("wheel_radius").value)),
            steering_limit=float(limits_cfg.get("max_steering_angle", 0.55)),
            steering_axle=str(kin_cfg.get("steering_axle", self.get_parameter("steering_axle").value)),
            drive_axle=str(kin_cfg.get("drive_axle", self.get_parameter("drive_axle").value)),
        )
        self.limits = ActuatorLimitConfig(**{
            field: float(limits_cfg.get(field, getattr(ActuatorLimitConfig(), field)))
            for field in ActuatorLimitConfig.__dataclass_fields__
        })

        self.last_twist = Twist()
        self.last_msg_time = None
        self.command = ActuatorCommand()
        self.last_update_time = self.get_clock().now()

        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._cmd_cb,
            10,
        )
        self.target_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("target_command_topic").value),
            10,
        )
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("steering_command_topic").value),
            10,
        )
        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("drive_command_topic").value),
            10,
        )
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self._on_timer)

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_twist = msg
        self.last_msg_time = self.get_clock().now()

    def _target_from_twist(self) -> ActuatorCommand:
        if self.last_msg_time is None:
            return ActuatorCommand()
        if (self.get_clock().now() - self.last_msg_time) > Duration(seconds=self.limits.command_timeout_sec):
            return ActuatorCommand()
        speed = float(self.last_twist.linear.x)
        steering = steering_angle_from_twist(speed, float(self.last_twist.angular.z), self.kinematics)
        return ActuatorCommand(speed=speed, steering_angle=steering)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        self.command = limit_command(self.command, self._target_from_twist(), dt, self.limits)

        target = Float64MultiArray()
        target.data = [self.command.speed, self.command.steering_angle]
        self.target_pub.publish(target)

        steering = Float64MultiArray()
        steering.data = [self.command.steering_angle, self.command.steering_angle]
        self.steering_pub.publish(steering)

        drive = Float64MultiArray()
        drive.data = [wheel_angular_velocity(self.command.speed, self.kinematics)] * 2
        self.drive_pub.publish(drive)


def main() -> None:
    rclpy.init()
    node = AckermannCommandAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
