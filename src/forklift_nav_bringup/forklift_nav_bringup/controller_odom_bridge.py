import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class ControllerOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("controller_odom_bridge")

        self.declare_parameter("odom_in_topic", "/rear_steer_controller/odometry")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("tf_in_topic", "")
        self.declare_parameter("tf_out_topic", "/tf")

        odom_in_topic = self.get_parameter("odom_in_topic").value
        odom_out_topic = self.get_parameter("odom_out_topic").value
        tf_in_topic = self.get_parameter("tf_in_topic").value
        tf_out_topic = self.get_parameter("tf_out_topic").value

        qos = QoSProfile(depth=20)
        self.odom_pub = self.create_publisher(Odometry, odom_out_topic, qos)
        self.create_subscription(Odometry, odom_in_topic, self._odom_callback, qos)
        self.tf_pub = None
        if tf_in_topic:
            self.tf_pub = self.create_publisher(TFMessage, tf_out_topic, qos)
            self.create_subscription(TFMessage, tf_in_topic, self._tf_callback, qos)

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_pub.publish(msg)

    def _tf_callback(self, msg: TFMessage) -> None:
        if self.tf_pub is not None:
            self.tf_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
