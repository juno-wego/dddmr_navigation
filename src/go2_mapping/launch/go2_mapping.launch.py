from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    lidar_source = LaunchConfiguration("lidar_source").perform(context)
    odom_source = LaunchConfiguration("odom_source").perform(context)

    package_share = get_package_share_path("go2_mapping")
    lego_loam_share = get_package_share_path("lego_loam_bor")

    if lidar_source == "livox":
        config_file = package_share / "config" / "go2_livox_mapping.yaml"
        pointcloud_topic = "/livox/lidar"
    else:
        config_file = package_share / "config" / "go2_utlidar_mapping.yaml"
        pointcloud_topic = "/go2/lidar_points"

    if odom_source == "aurora":
        odom_topic = "/slamware_ros_sdk_server_node/odom"
    else:
        odom_topic = "/odom"

    rviz_config = lego_loam_share / "rviz" / "lego_loam.rviz"

    return [
        Node(
            package="lego_loam_bor",
            executable="lego_loam",
            name="lego_loam",
            output="screen",
            parameters=[str(config_file)],
            remappings=[
                ("lslidar_point_cloud", pointcloud_topic),
                ("odom", odom_topic),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="go2_mapping_rviz",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("show_rviz")),
        ),
    ]


def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
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
            "odom_source": LaunchConfiguration("odom_source"),
            "use_go2": LaunchConfiguration("use_go2"),
            "use_aurora": LaunchConfiguration("use_aurora"),
            "use_mid360": LaunchConfiguration("use_mid360"),
            "use_go2_camera": LaunchConfiguration("use_go2_camera"),
            "show_rviz": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="enp88s0"),
            DeclareLaunchArgument("aurora_ip", default_value="192.168.11.1"),
            DeclareLaunchArgument(
                "odom_source",
                default_value="aurora",
                choices=["aurora", "go2"],
                description="Odometry source used by lego_loam.",
            ),
            DeclareLaunchArgument(
                "lidar_source",
                default_value="utlidar",
                choices=["utlidar", "livox"],
                description="Point cloud source used by lego_loam.",
            ),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="false"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            bringup_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
