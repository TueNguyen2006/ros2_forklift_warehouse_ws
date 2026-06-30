#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRetimestamp(Node):
    def __init__(self) -> None:
        super().__init__("scan_retimestamp")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_visual")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(LaserScan, output_topic, qos)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.on_scan,
            qos,
        )
        self.get_logger().info(
            f"Republishing {input_topic} to {output_topic} with current ROS time stamps"
        )

    def on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanRetimestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
