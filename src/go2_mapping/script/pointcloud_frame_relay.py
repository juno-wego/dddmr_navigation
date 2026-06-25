#!/usr/bin/env python3

from contextlib import suppress

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf_transformations
import tf2_ros


class PointCloudFrameRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_relay")
        self.declare_parameter("input_topic", "/go2/lidar_points")
        self.declare_parameter("output_topic", "/go2/lidar_points_mapping")
        self.declare_parameter("output_frame", "go2_mapping_lidar")
        self.declare_parameter("transform_points", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.transform_points = self.get_parameter("transform_points").value

        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, output_topic, output_qos)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, input_qos
        )
        mode = "transforming points" if self.transform_points else "rewriting frame only"
        self.get_logger().info(
            f"Relaying {input_topic} to {output_topic} in frame_id={self.output_frame} ({mode})"
        )

    def callback(self, msg):
        if not self.transform_points:
            # Relabeling a cloud from another coordinate system corrupts both
            # mapping and localization silently.  The merger is configured to
            # output in base_link, so require that contract here.
            input_frame = msg.header.frame_id.lstrip("/")
            output_frame = self.output_frame.lstrip("/")
            if input_frame != output_frame:
                self.get_logger().error(
                    f"Refusing to relabel cloud {msg.header.frame_id} as "
                    f"{self.output_frame} without transforming points.",
                    throttle_duration_sec=2.0,
                )
                return
            out = PointCloud2()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.output_frame
            out.height = msg.height
            out.width = msg.width
            out.fields = msg.fields
            out.is_bigendian = msg.is_bigendian
            out.point_step = msg.point_step
            out.row_step = msg.row_step
            out.data = msg.data
            out.is_dense = msg.is_dense
            self.publisher.publish(out)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.02),
            )
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Waiting for TF {self.output_frame} <- {msg.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return

        points = point_cloud2.read_points(msg, skip_nans=False).copy()
        if points.dtype.names is None or not {"x", "y", "z"}.issubset(points.dtype.names):
            self.get_logger().warn("PointCloud2 has no x/y/z fields.", throttle_duration_sec=2.0)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = tf_transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        xyz = np.stack(
            [
                points["x"].astype(np.float64, copy=False),
                points["y"].astype(np.float64, copy=False),
                points["z"].astype(np.float64, copy=False),
            ],
            axis=0,
        )
        xyz = matrix[:3, :3].dot(xyz)
        points["x"] = xyz[0] + translation.x
        points["y"] = xyz[1] + translation.y
        points["z"] = xyz[2] + translation.z

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = points.dtype.itemsize
        out.row_step = out.point_step * out.width
        out.data = points.tobytes()
        out.is_dense = msg.is_dense
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameRelay()
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
