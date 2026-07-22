import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from forklift_control.kinematics import ForkliftKinematicConfig, yaw_rate_from_state


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def integrate_odometry(
    state: OdomState,
    wheel_speed_rad_s: float,
    steering_angle: float,
    dt: float,
    config: ForkliftKinematicConfig,
) -> OdomState:
    speed = wheel_speed_rad_s * config.wheel_radius
    yaw_rate = yaw_rate_from_state(speed, steering_angle, config)
    mid_yaw = state.yaw + 0.5 * yaw_rate * dt
    return OdomState(
        x=state.x + speed * math.cos(mid_yaw) * dt,
        y=state.y + speed * math.sin(mid_yaw) * dt,
        yaw=state.yaw + yaw_rate * dt,
        speed=speed,
        yaw_rate=yaw_rate,
    )


class WheelOdometry(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odometry")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("left_drive_joint", "left_front_wheel_rotation_joint")
        self.declare_parameter("right_drive_joint", "right_front_wheel_rotation_joint")
        self.declare_parameter("left_steering_joint", "left_rear_steering_joint")
        self.declare_parameter("right_steering_joint", "right_rear_steering_joint")
        self.declare_parameter("wheelbase", 1.35)
        self.declare_parameter("wheel_radius", 0.16)
        self.declare_parameter("steering_axle", "rear")
        self.declare_parameter("drive_axle", "front")

        self.config = ForkliftKinematicConfig(
            wheelbase=float(self.get_parameter("wheelbase").value),
            wheel_radius=float(self.get_parameter("wheel_radius").value),
            steering_axle=str(self.get_parameter("steering_axle").value),
            drive_axle=str(self.get_parameter("drive_axle").value),
        )
        self.state = OdomState()
        self.last_stamp = None
        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_cb,
            20,
        )

    def _joint_value(self, msg: JointState, name: str, values: list[float]) -> float | None:
        try:
            index = msg.name.index(name)
        except ValueError:
            return None
        if index >= len(values):
            return None
        return float(values[index])

    def _joint_cb(self, msg: JointState) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = max((stamp - self.last_stamp).nanoseconds / 1e9, 0.0)
        self.last_stamp = stamp
        if dt <= 0.0:
            return

        left_speed = self._joint_value(msg, str(self.get_parameter("left_drive_joint").value), list(msg.velocity))
        right_speed = self._joint_value(msg, str(self.get_parameter("right_drive_joint").value), list(msg.velocity))
        left_steering = self._joint_value(msg, str(self.get_parameter("left_steering_joint").value), list(msg.position))
        right_steering = self._joint_value(msg, str(self.get_parameter("right_steering_joint").value), list(msg.position))
        if None in (left_speed, right_speed, left_steering, right_steering):
            return

        wheel_speed = 0.5 * (left_speed + right_speed)
        steering_angle = 0.5 * (left_steering + right_steering)
        self.state = integrate_odometry(self.state, wheel_speed, steering_angle, dt, self.config)
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = str(self.get_parameter("odom_frame").value)
        odom.child_frame_id = str(self.get_parameter("base_frame").value)
        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.state.yaw)
        odom.twist.twist.linear.x = self.state.speed
        odom.twist.twist.angular.z = self.state.yaw_rate
        self.odom_pub.publish(odom)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = self.state.x
            transform.transform.translation.y = self.state.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
