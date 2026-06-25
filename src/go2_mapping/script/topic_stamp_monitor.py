#!/usr/bin/env python3

from contextlib import suppress

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2


class TopicStampMonitor(Node):
    def __init__(self):
        super().__init__("go2_topic_stamp_monitor")
        self.declare_parameter("cloud_topics", ["/go2/lidar_points", "/livox/lidar", "/merged/points"])
        self.declare_parameter("odom_topics", ["/slamware_ros_sdk_server_node/odom"])
        self.declare_parameter("report_rate", 1.0)
        self.declare_parameter("warn_age", 0.2)
        self.declare_parameter("warn_skew", 0.12)

        self.warn_age = float(self.get_parameter("warn_age").value)
        self.warn_skew = float(self.get_parameter("warn_skew").value)
        self.latest = {}

        cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._topic_subscriptions = []
        for topic in self.get_parameter("cloud_topics").value:
            self._topic_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, topic=topic: self.update(topic, msg.header.stamp),
                    cloud_qos,
                )
            )
        for topic in self.get_parameter("odom_topics").value:
            self._topic_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, topic=topic: self.update(topic, msg.header.stamp),
                    odom_qos,
                )
            )

        period = 1.0 / max(1e-3, float(self.get_parameter("report_rate").value))
        self.timer = self.create_timer(period, self.report)

    @staticmethod
    def stamp_to_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def update(self, topic, stamp):
        self.latest[topic] = self.stamp_to_ns(stamp)

    def report(self):
        if not self.latest:
            self.get_logger().info("Waiting for configured topics...")
            return

        now_ns = self.get_clock().now().nanoseconds
        valid = {topic: stamp_ns for topic, stamp_ns in self.latest.items() if stamp_ns > 0}
        newest_ns = max(valid.values()) if valid else 0
        parts = []
        warn = False
        for topic in sorted(self.latest):
            stamp_ns = self.latest[topic]
            if stamp_ns <= 0:
                parts.append(f"{topic}: stamp=0")
                continue
            age = (now_ns - stamp_ns) / 1e9
            skew = (newest_ns - stamp_ns) / 1e9 if newest_ns > 0 else 0.0
            warn = warn or age > self.warn_age or skew > self.warn_skew
            parts.append(f"{topic}: age={age:.3f}s skew={skew:.3f}s")

        message = " | ".join(parts)
        if warn:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = TopicStampMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
