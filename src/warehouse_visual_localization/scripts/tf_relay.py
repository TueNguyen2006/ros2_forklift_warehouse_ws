#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self) -> None:
        super().__init__("tf_relay")

        self.declare_parameter("input_topic", "/rear_steer_controller/tf_odometry")
        self.declare_parameter("output_topic", "/tf")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(TFMessage, output_topic, 50)
        self.subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self._handle_tf,
            50,
        )

        self.get_logger().info(f"Relaying TF {input_topic} -> {output_topic}")

    def _handle_tf(self, message: TFMessage) -> None:
        if message.transforms:
            self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = TfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
