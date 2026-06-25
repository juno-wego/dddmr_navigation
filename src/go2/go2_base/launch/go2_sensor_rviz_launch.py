from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_rviz_node(context, *args, **kwargs):
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    go2_description_path = get_package_share_path("go2_description")
    rviz_template_path = go2_description_path / "rviz/urdf.rviz"
    rviz_config = Path(f"/tmp/go2_sensor_rviz_{fixed_frame}.rviz")

    text = rviz_template_path.read_text()
    lines = []
    for line in text.splitlines():
        if line.strip().startswith("Fixed Frame:"):
            indent = line[: len(line) - len(line.lstrip())]
            lines.append(f"{indent}Fixed Frame: {fixed_frame}")
        else:
            lines.append(line)
    rviz_config.write_text("\n".join(lines) + "\n")

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    ]


def generate_launch_description():
    go2_base_path = get_package_share_path("go2_base")

    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(go2_base_path / "launch/go2_bringup_launch.py")),
        launch_arguments={
            "interface": LaunchConfiguration("interface"),
            "gui": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="enp88s0",
                description="Network interface for Unitree SDK.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="base_link",
                description="RViz fixed frame. Use utlidar to inspect the raw cloud orientation.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz.",
            ),
            go2_bringup,
            OpaqueFunction(function=make_rviz_node),
        ]
    )
