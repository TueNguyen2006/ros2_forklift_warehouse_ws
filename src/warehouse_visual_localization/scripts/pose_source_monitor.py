#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSourceMonitor(Node):
    def __init__(self):
        super().__init__("pose_source_monitor")
        self.declare_parameter("pose_source", "rgbd_odom")
        self.declare_parameter("consumer_name", "visual_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("track_map_frame", True)
        self.declare_parameter("require_map_frame", False)
        self.declare_parameter("period_sec", 2.0)
        self.declare_parameter("required_odom_topics_csv", "")
        self.declare_parameter("topic_timeout_sec", 3.0)

        self.pose_source = str(self.get_parameter("pose_source").value)
        self.consumer_name = str(self.get_parameter("consumer_name").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.track_map_frame = bool(self.get_parameter("track_map_frame").value)
        self.require_map_frame = bool(self.get_parameter("require_map_frame").value)
        period = float(self.get_parameter("period_sec").value)
        required_topics_csv = str(
            self.get_parameter("required_odom_topics_csv").value
        )
        self.required_odom_topics = [
            topic.strip()
            for topic in required_topics_csv.split(",")
            if topic.strip()
        ]
        self.topic_timeout = Duration(
            seconds=float(self.get_parameter("topic_timeout_sec").value)
        )

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.last_topic_msg_time: dict[str, rclpy.time.Time] = {}
        self._topic_subscriptions = [
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic_name=topic: self._odom_cb(topic_name, msg),
                10,
            )
            for topic in self.required_odom_topics
        ]
        self.create_timer(period, self._on_timer)

    def _odom_cb(self, topic_name: str, _msg: Odometry) -> None:
        self.last_topic_msg_time[topic_name] = self.get_clock().now()

    def _has_transform(self, target: str, source: str) -> bool:
        try:
            self.buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
            return True
        except TransformException:
            return False

    def _required_topics_status(self) -> tuple[bool, str]:
        if not self.required_odom_topics:
            return True, "disabled"

        now = self.get_clock().now()
        missing = []
        stale = []

        for topic in self.required_odom_topics:
            last_msg_time = self.last_topic_msg_time.get(topic)
            if last_msg_time is None:
                missing.append(topic)
                continue

            if now - last_msg_time > self.topic_timeout:
                stale.append(topic)

        if missing:
            return False, "missing:" + ",".join(missing)
        if stale:
            return False, "stale:" + ",".join(stale)
        return True, "ok"

    def _on_timer(self):
        has_odom = self._has_transform(self.odom_frame, self.robot_frame)
        has_map = (
            self._has_transform(self.global_frame, self.odom_frame)
            if self.track_map_frame
            else False
        )
        has_topics, topics_status = self._required_topics_status()
        state = (
            "ready"
            if has_odom
            and has_topics
            and ((has_map and self.track_map_frame) or not self.require_map_frame)
            else ("waiting_for_topics" if not has_topics else "waiting_for_tf")
        )
        self.get_logger().info(
            "Pose source=%s | consumer=%s | state=%s | source_topics=%s | map->odom=%s | odom->base=%s"
            % (
                self.pose_source,
                self.consumer_name,
                state,
                topics_status,
                (
                    "disabled"
                    if not self.track_map_frame
                    else ("ok" if has_map else "missing")
                ),
                "ok" if has_odom else "missing",
            )
        )


def main():
    rclpy.init()
    node = PoseSourceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
