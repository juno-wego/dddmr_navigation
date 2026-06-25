from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


SENSOR_BAG_TOPICS = [
    "/aurora_robot_pose",
    "/go2/camera/camera_info",
    "/go2/camera/image_raw",
    "/go2/camera/image_raw/compressed",
    "/go2/comm_ok",
    "/go2/faults_raw",
    "/go2/imu",
    "/go2/lidar_points",
    "/go2/tf",
    "/go2/wheel_odom",
    "/go2_battery_state",
    "/go2_mode",
    "/go2_motor_state",
    "/joint_states",
    "/livox/imu",
    "/livox/lidar",
    "/map_updates",
    "/robot_description",
    "/slamware_ros_sdk_server_node/depth_image_colorized",
    "/slamware_ros_sdk_server_node/depth_image_raw",
    "/slamware_ros_sdk_server_node/imu_raw_data",
    "/slamware_ros_sdk_server_node/left_image_raw",
    "/slamware_ros_sdk_server_node/map",
    "/slamware_ros_sdk_server_node/map_metadata",
    "/slamware_ros_sdk_server_node/odom",
    "/slamware_ros_sdk_server_node/point_cloud",
    "/slamware_ros_sdk_server_node/right_image_raw",
    "/slamware_ros_sdk_server_node/scan",
    "/slamware_ros_sdk_server_node/semantic_segmentation",
    "/slamware_ros_sdk_server_node/state",
    "/slamware_ros_sdk_server_node/stereo_keypoints",
    "/slamware_ros_sdk_server_node/system_status",
    "/tf",
    "/tf_static",
]


def make_bag_recorder(context, *args, **kwargs):
    output_dir = Path(LaunchConfiguration("output_dir").perform(context)).expanduser()
    bag_name = LaunchConfiguration("bag_name").perform(context)
    storage_id = LaunchConfiguration("storage_id").perform(context)
    max_bag_size = LaunchConfiguration("max_bag_size").perform(context)
    use_compression = LaunchConfiguration("use_compression").perform(context).lower() == "true"

    if bag_name == "auto":
        bag_name = "sensors_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    output_dir.mkdir(parents=True, exist_ok=True)
    bag_path = output_dir / bag_name
    cmd = [
        "ros2",
        "bag",
        "record",
        "--storage",
        storage_id,
        "--output",
        str(bag_path),
        "--max-bag-size",
        max_bag_size,
    ]

    if use_compression:
        cmd += ["--compression-mode", "file", "--compression-format", "zstd"]

    cmd += SENSOR_BAG_TOPICS

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
        )
    ]


def generate_launch_description():
    default_output_dir = str(Path.home() / "dddmr_navigation" / "bags")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=default_output_dir,
                description="Directory where sensor rosbag folders are created.",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value="auto",
                description="Bag folder name. Use 'auto' for sensors_YYYYmmdd_HHMMSS.",
            ),
            DeclareLaunchArgument(
                "storage_id",
                default_value="sqlite3",
                description="rosbag2 storage plugin, usually sqlite3 or mcap if installed.",
            ),
            DeclareLaunchArgument(
                "max_bag_size",
                default_value="4294967296",
                description="Split bag files after this many bytes.",
            ),
            DeclareLaunchArgument(
                "use_compression",
                default_value="false",
                choices=["true", "false"],
                description="Enable zstd file compression.",
            ),
            OpaqueFunction(function=make_bag_recorder),
        ]
    )
