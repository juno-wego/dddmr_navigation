"""Localization-only Go2 launch.

Defaults to the stable real-robot path:
- direct Livox cloud into MCL
- planner/controller disabled
- Go2 internal odometry selected by default

Only the odometry source is expected to change during normal use:
    ros2 launch go2_navigation go2_localization.launch.py odom_source:=go2
    ros2 launch go2_navigation go2_localization.launch.py odom_source:=aurora
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_dir", default_value=""),
            DeclareLaunchArgument(
                "map_root",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "dddmr_navigation", "maps", "go2"]
                ),
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="go2",
                choices=["aurora", "go2"],
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("go2_navigation"),
                            "launch",
                            "go2_navigation.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "map_dir": LaunchConfiguration("map_dir"),
                    "map_root": LaunchConfiguration("map_root"),
                    "odom_source": LaunchConfiguration("odom_source"),
                    "use_go2": "true",
                    "use_aurora": "true",
                    "use_mid360": "true",
                    "use_go2_camera": "false",
                    "rviz": LaunchConfiguration("rviz"),
                    "lidar_source": "livox",
                    "start_navigation": "false",
                }.items(),
            ),
        ]
    )
