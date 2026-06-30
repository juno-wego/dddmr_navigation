#!/usr/bin/env python3
# Copyright (c) 2024 wego
# SPDX-License-Identifier: MIT
"""
save_go2_map.py
---------------
Call the `lego_loam_mo/save_mapped_point_cloud` ROS 2 service that is
advertised by lego_loam_bor while the mapping node is alive.

The pose-graph directory is resolved by the DDDMR_MAPPING_DIR environment
variable (set by go2_mapping.launch.py) plus a timestamp sub-folder, so
the save destination is printed **before** the call so you know where to
look when relaunching for navigation.

Usage (in the navigation container after sourcing):
    ros2 run go2_mapping save_go2_map.py
"""

import sys
import argparse
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


# The lego_loam_bor mapOptimization node lives in the default namespace;
# the service name matches what is created in mapOptimization.cpp.
MAP_SAVE_SERVICE = "/save_mapped_point_cloud"
TIMEOUT_SEC = 10.0
CALL_TIMEOUT_SEC = 600.0
REQUIRED_MAP_FILES = ("poses.pcd", "edges.pcd", "map.pcd", "ground.pcd")


def default_map_root() -> Path:
    env_value = os.environ.get("DDDMR_MAPPING_DIR")
    if env_value:
        return Path(env_value).expanduser()
    return Path.home() / "dddmr_navigation" / "maps" / "go2"


def is_pose_graph_dir(path: Path) -> bool:
    return path.is_dir() and all((path / name).exists() for name in REQUIRED_MAP_FILES)


def find_latest_pose_graph(root: Path, newer_than: float | None = None) -> Path | None:
    if not root.exists():
        return None

    candidates = []
    for child in root.iterdir():
        if not is_pose_graph_dir(child):
            continue
        mtime = child.stat().st_mtime
        if newer_than is not None and mtime < newer_than:
            continue
        candidates.append((mtime, child))

    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


class MapSaverNode(Node):
    def __init__(self, map_root: Path):
        super().__init__("go2_map_saver")
        self._map_root = map_root
        self._client = self.create_client(Empty, MAP_SAVE_SERVICE)

    def save(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=TIMEOUT_SEC):
            self.get_logger().error(
                f"Service '{MAP_SAVE_SERVICE}' not available after "
                f"{TIMEOUT_SEC:.0f} s. Is the mapping node running?"
            )
            return False

        start_time = time.time()
        self.get_logger().info(
            "Calling map save service. The mapping node writes under:\n"
            f"  {self._map_root}"
        )

        future = self._client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)

        if future.result() is None:
            self.get_logger().error("Map save call timed-out or was interrupted.")
            return False

        saved_dir = find_latest_pose_graph(self._map_root, newer_than=start_time - 1.0)
        if saved_dir is None:
            saved_dir = find_latest_pose_graph(self._map_root)

        if saved_dir is None:
            self.get_logger().warning(
                "Map save service returned, but no complete pose-graph directory "
                f"was found under {self._map_root}.\n"
                "Check the lego_loam_mo log for the exact 'Create dir:' line."
            )
            return True

        self.get_logger().info(
            "Map save service returned successfully.\n"
            f"Saved pose-graph directory:\n  {saved_dir}\n\n"
            "To load this map for navigation use:\n"
            "  ros2 launch go2_navigation go2_navigation.launch.py \\\n"
            f"    map_dir:={saved_dir}"
        )
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map-root",
        default=str(default_map_root()),
        help=(
            "Directory where go2_mapping.launch.py stores timestamped maps. "
            "Default: $DDDMR_MAPPING_DIR or ~/dddmr_navigation/maps/go2"
        ),
    )
    args, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = MapSaverNode(Path(args.map_root).expanduser())
    try:
        ok = node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
