from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_share = get_package_share_path("go2_mapping")
    mapping_dir = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "dddmr_navigation", "maps", "go2"]
    )

    sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("go2_bringup"),
                    "launch",
                    "all_sensors_rviz.launch.py",
                ]
            )
        ),
        launch_arguments={
            "interface": LaunchConfiguration("interface"),
            "aurora_ip": LaunchConfiguration("aurora_ip"),
            "odom_source": "aurora",
            "use_go2": LaunchConfiguration("use_go2"),
            "use_aurora": LaunchConfiguration("use_aurora"),
            "use_mid360": LaunchConfiguration("use_mid360"),
            "use_go2_camera": LaunchConfiguration("use_go2_camera"),
            "lidar_frame_id": "utlidar",
            "show_rviz": "false",
        }.items(),
    )

    mapping_cloud_transform = Node(
        package="go2_mapping",
        executable="pointcloud_frame_relay.py",
        name="go2_mapping_cloud_transform",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("raw_lidar_topic"),
                "output_topic": LaunchConfiguration("lidar_topic"),
                "output_frame": "base_footprint",
                "transform_points": True,
            }
        ],
    )

    pointcloud_merger = Node(
        package="go2_mapping",
        executable="pointcloud_merger.py",
        name="go2_pointcloud_merger",
        output="screen",
        parameters=[
            {
                "input_topics": ["/go2/lidar_points", "/livox/lidar"],
                "output_topic": "/merged/points",
                "output_frame": "base_footprint",
                "publish_rate": 10.0,
                "max_cloud_age": 0.2,
                "max_stamp_skew": 0.12,
            }
        ],
    )

    lego_loam = Node(
        package="lego_loam_bor",
        executable="lego_loam",
        output="screen",
        parameters=[
            str(mapping_share / "config/go2_real_mapping.yaml"),
            {
                "use_sim_time": False,
                "laser.base_ground_frame": "base_footprint",
            },
        ],
        remappings=[
            ("lslidar_point_cloud", LaunchConfiguration("lidar_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_mapping_rviz",
        output="screen",
        arguments=["-d", str(mapping_share / "rviz/go2_real_mapping.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            SetEnvironmentVariable("DDDMR_MAPPING_DIR", [mapping_dir, "/"]),
            DeclareLaunchArgument("interface", default_value="enp88s0"),
            DeclareLaunchArgument("aurora_ip", default_value="192.168.11.1"),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("raw_lidar_topic", default_value="/merged/points"),
            DeclareLaunchArgument("lidar_topic", default_value="/go2/lidar_points_base"),
            DeclareLaunchArgument(
                "odom_topic", default_value="/slamware_ros_sdk_server_node/odom"
            ),
            LogInfo(
                msg=[
                    "\n\n",
                    "╔══════════════════════════════════════════════════════════╗\n",
                    "║            GO2 MAPPING  —  MAP SAVE INSTRUCTIONS        ║\n",
                    "╠══════════════════════════════════════════════════════════╣\n",
                    "║  While mapping is running, call in a new terminal:       ║\n",
                    "║                                                          ║\n",
                    "║    ros2 run go2_mapping save_go2_map.py                  ║\n",
                    "║                                                          ║\n",
                    "║  Map will be saved to:                                   ║\n",
                    "║    ~/dddmr_navigation/maps/go2/<timestamp>/              ║\n",
                    "║                                                          ║\n",
                    "║  Then navigate using:                                    ║\n",
                    "║    ros2 launch go2_mapping go2_navigation.launch.py \\    ║\n",
                    "║        map_dir:=<path above>                             ║\n",
                    "╚══════════════════════════════════════════════════════════╝\n",
                ]
            ),
            sensors_bringup,
            pointcloud_merger,
            mapping_cloud_transform,
            lego_loam,
            rviz,
        ]
    )
