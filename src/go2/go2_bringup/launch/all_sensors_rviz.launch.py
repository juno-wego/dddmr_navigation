from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="enp88s0"),
            DeclareLaunchArgument("aurora_ip", default_value="192.168.11.1"),
            DeclareLaunchArgument("odom_source", default_value="aurora", choices=["aurora", "go2"]),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("go2_bringup"),
                            "launch",
                            "go2_aurora_rviz.launch.py",
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
                    "show_rviz": LaunchConfiguration("show_rviz"),
                }.items(),
            )
        ]
    )
