from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class OdomToImu(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_imu")
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("output_imu_topic", "/imu")

        input_odom_topic = (
            self.get_parameter("input_odom_topic").get_parameter_value().string_value
        )
        output_imu_topic = (
            self.get_parameter("output_imu_topic").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(Imu, output_imu_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, input_odom_topic, self.odom_callback, 20
        )

        self.last_linear_x: Optional[float] = None
        self.last_stamp = None

    def odom_callback(self, msg: Odometry) -> None:
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation = msg.pose.pose.orientation
        imu_msg.angular_velocity = msg.twist.twist.angular

        if self.last_stamp is not None and self.last_linear_x is not None:
            dt = (msg.header.stamp.sec - self.last_stamp.sec) + (
                (msg.header.stamp.nanosec - self.last_stamp.nanosec) / 1e9
            )
            if dt > 1e-6:
                imu_msg.linear_acceleration.x = (
                    msg.twist.twist.linear.x - self.last_linear_x
                ) / dt
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.orientation_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04]
        imu_msg.angular_velocity_covariance = [
            0.02,
            0.0,
            0.0,
            0.0,
            0.02,
            0.0,
            0.0,
            0.0,
            0.04,
        ]
        imu_msg.linear_acceleration_covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.2,
        ]

        self.publisher.publish(imu_msg)
        self.last_linear_x = msg.twist.twist.linear.x
        self.last_stamp = msg.header.stamp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
