import rclpy
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GroundTruthOdom(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_odom")
        self.declare_parameter("entity_name", "forklift_physics")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        self.declare_parameter("publish_hz", 20.0)
        self.publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )
        self.client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.create_timer(1.0 / max(float(self.get_parameter("publish_hz").value), 1.0), self._on_timer)

    def _on_timer(self) -> None:
        if not self.client.service_is_ready():
            return
        request = GetEntityState.Request()
        request.name = str(self.get_parameter("entity_name").value)
        request.reference_frame = str(self.get_parameter("reference_frame").value)
        future = self.client.call_async(request)
        future.add_done_callback(self._publish_response)

    def _publish_response(self, future) -> None:
        response = future.result()
        if response is None or not response.success:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose = response.state.pose
        odom.twist.twist = response.state.twist
        self.publisher.publish(odom)


def main() -> None:
    rclpy.init()
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
