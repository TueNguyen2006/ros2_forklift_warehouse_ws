#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay")
        self.declare_parameter("input_topic", "/rear_steer_controller/odometry")
        self.declare_parameter("output_topic", "/odom")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._odom_cb, 20
        )

        self.get_logger().info("Relaying odom %s -> %s" % (input_topic, output_topic))

    def _odom_cb(self, msg: Odometry) -> None:
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
