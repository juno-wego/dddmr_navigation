import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_bag_launch(context, *args, **kwargs):
    lidar_source = LaunchConfiguration("lidar_source").perform(context)
    odom_source = LaunchConfiguration("odom_source").perform(context)
    bag_file_dir = LaunchConfiguration("bag_file_dir").perform(context)

    package_share = get_package_share_path("go2_mapping")
    lego_loam_share = get_package_share_path("lego_loam_bor")

    if lidar_source == "livox":
        config_file = package_share / "config" / "go2_livox_bag.yaml"
        pointcloud_topic = "/livox/lidar"
    else:
        config_file = package_share / "config" / "go2_utlidar_bag.yaml"
        pointcloud_topic = "/go2/lidar_points"

    if odom_source == "aurora":
        odom_topic = "/slamware_ros_sdk_server_node/odom"
    else:
        odom_topic = "/odom"

    with config_file.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    config["bag_reader"]["ros__parameters"]["bag_file_dir"] = bag_file_dir
    config["bag_reader"]["ros__parameters"]["point_cloud_topic"] = pointcloud_topic
    config["bag_reader"]["ros__parameters"]["odometry_topic"] = odom_topic

    temp_config = Path(
        tempfile.gettempdir(),
        f"go2_mapping_bag_{lidar_source}_{odom_source}.yaml",
    )
    with temp_config.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

    rviz_config = lego_loam_share / "rviz" / "lego_loam.rviz"

    return [
        Node(
            package="lego_loam_bor",
            executable="lego_loam_bag",
            name="lego_loam_bag",
            output="screen",
            parameters=[str(temp_config)],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="go2_mapping_bag_rviz",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("show_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_file_dir",
                default_value="/root/dddmr_bags/go2",
                description="Path to the rosbag directory.",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="aurora",
                choices=["aurora", "go2"],
                description="Recorded odometry topic family inside the bag.",
            ),
            DeclareLaunchArgument(
                "lidar_source",
                default_value="utlidar",
                choices=["utlidar", "livox"],
                description="Recorded point cloud topic inside the bag.",
            ),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            OpaqueFunction(function=build_bag_launch),
        ]
    )
