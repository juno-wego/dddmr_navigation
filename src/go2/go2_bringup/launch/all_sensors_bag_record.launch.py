from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_dir", default_value="~/dddmr_navigation/bags"),
            DeclareLaunchArgument("bag_name", default_value="auto"),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("max_bag_size", default_value="4294967296"),
            DeclareLaunchArgument("use_compression", default_value="false", choices=["true", "false"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("go2_bringup"),
                            "launch",
                            "go2_bag_record.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "output_dir": LaunchConfiguration("output_dir"),
                    "bag_name": LaunchConfiguration("bag_name"),
                    "storage_id": LaunchConfiguration("storage_id"),
                    "max_bag_size": LaunchConfiguration("max_bag_size"),
                    "use_compression": LaunchConfiguration("use_compression"),
                }.items(),
            ),
        ]
    )
