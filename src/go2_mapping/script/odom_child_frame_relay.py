#!/usr/bin/env python3

from contextlib import suppress

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class OdomChildFrameRelay(Node):
    def __init__(self):
        super().__init__("odom_child_frame_relay")
        self.declare_parameter("input_topic", "/slamware_ros_sdk_server_node/odom")
        self.declare_parameter("output_topic", "/slamware_ros_sdk_server_node/odom_base_link")
        self.declare_parameter("child_frame_id", "base_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(Odometry, output_topic, qos)
        self.subscription = self.create_subscription(Odometry, input_topic, self.callback, qos)
        self.get_logger().info(
            f"Relaying {input_topic} to {output_topic} with child_frame_id={self.child_frame_id}"
        )

    def callback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = self.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomChildFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
