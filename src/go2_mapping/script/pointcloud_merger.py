#!/usr/bin/env python3

from contextlib import suppress

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf_transformations
import tf2_ros


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__("pointcloud_merger")
        self.declare_parameter("input_topics", ["/go2/lidar_points", "/livox/lidar"])
        self.declare_parameter("output_topic", "/merged/points")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("max_cloud_age", 0.2)
        self.declare_parameter("max_stamp_skew", 0.12)

        self.input_topics = list(self.get_parameter("input_topics").value)
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.max_cloud_age = float(self.get_parameter("max_cloud_age").value)
        self.max_stamp_skew = float(self.get_parameter("max_stamp_skew").value)
        self.last_published_stamp_ns = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        out_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest = {}
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, out_qos)
        self.cloud_subscriptions = [
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic=topic: self.cloud_callback(topic, msg),
                qos,
            )
            for topic in self.input_topics
        ]

        period = 1.0 / max(1e-3, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(period, self.publish_merged)
        self.get_logger().info(
            f"Merging {self.input_topics} into {self.output_topic} in {self.output_frame}"
        )

    def cloud_callback(self, topic, msg):
        self.latest[topic] = msg

    def lookup_matrix(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.02),
            )
        except Exception:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.02),
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = tf_transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        return matrix

    def cloud_to_xyzi(self, msg):
        points = point_cloud2.read_points(msg, field_names=None, skip_nans=True)
        if points.dtype.names is None or not {"x", "y", "z"}.issubset(points.dtype.names):
            return None

        xyz = np.stack(
            [
                points["x"].astype(np.float64, copy=False),
                points["y"].astype(np.float64, copy=False),
                points["z"].astype(np.float64, copy=False),
                np.ones(points.shape[0], dtype=np.float64),
            ],
            axis=0,
        )
        matrix = self.lookup_matrix(msg)
        xyz = matrix.dot(xyz)

        if "intensity" in points.dtype.names:
            intensity = points["intensity"].astype(np.float32, copy=False)
        elif "reflectivity" in points.dtype.names:
            intensity = points["reflectivity"].astype(np.float32, copy=False)
        else:
            intensity = np.zeros(points.shape[0], dtype=np.float32)

        return np.column_stack(
            [
                xyz[0].astype(np.float32),
                xyz[1].astype(np.float32),
                xyz[2].astype(np.float32),
                intensity,
            ]
        )

    def stamp_age(self, stamp: TimeMsg):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return 0.0
        now = self.get_clock().now()
        msg_time = Time.from_msg(stamp)
        return (now - msg_time).nanoseconds / 1e9

    @staticmethod
    def stamp_to_ns(stamp: TimeMsg):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def publish_merged(self):
        merged = []
        used_topics = []
        candidates = []
        for topic, msg in list(self.latest.items()):
            if self.stamp_age(msg.header.stamp) > self.max_cloud_age:
                continue
            candidates.append((topic, msg, self.stamp_to_ns(msg.header.stamp)))

        if not candidates:
            return

        valid_stamp_ns = [stamp_ns for _, _, stamp_ns in candidates if stamp_ns > 0]
        newest_stamp_ns = max(valid_stamp_ns) if valid_stamp_ns else 0
        if self.last_published_stamp_ns == newest_stamp_ns and newest_stamp_ns > 0:
            return

        for topic, msg, stamp_ns in candidates:
            if (
                newest_stamp_ns > 0
                and stamp_ns > 0
                and self.max_stamp_skew > 0.0
                and (newest_stamp_ns - stamp_ns) / 1e9 > self.max_stamp_skew
            ):
                self.get_logger().warn(
                    f"Skipping stale cloud {topic}: stamp skew "
                    f"{(newest_stamp_ns - stamp_ns) / 1e9:.3f}s > "
                    f"{self.max_stamp_skew:.3f}s",
                    throttle_duration_sec=2.0,
                )
                continue
            try:
                xyzi = self.cloud_to_xyzi(msg)
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping {topic}: TF {self.output_frame} <- {msg.header.frame_id} unavailable: {exc}",
                    throttle_duration_sec=2.0,
                )
                continue
            if xyzi is None or xyzi.size == 0:
                continue
            merged.append(xyzi)
            used_topics.append(topic)

        if not merged:
            return

        data = np.concatenate(merged, axis=0)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = PointCloud2().header
        header.stamp = (
            Time(nanoseconds=newest_stamp_ns).to_msg()
            if newest_stamp_ns > 0
            else self.get_clock().now().to_msg()
        )
        header.frame_id = self.output_frame
        cloud = point_cloud2.create_cloud(header, fields, data)
        self.publisher.publish(cloud)
        self.last_published_stamp_ns = newest_stamp_ns
        self.get_logger().debug(f"Published merged cloud from {used_topics}: {data.shape[0]}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
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
