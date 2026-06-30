from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


REQUIRED_MAP_FILES = ("map.pcd", "ground.pcd")


def _resolve_map_dir(map_dir_value: str, map_root_value: str) -> Path:
    if map_dir_value:
        resolved = Path(map_dir_value).expanduser()
    else:
        root = Path(map_root_value).expanduser()
        candidates = [
            child for child in root.iterdir()
            if child.is_dir() and all((child / name).exists() for name in REQUIRED_MAP_FILES)
        ] if root.exists() else []
        if not candidates:
            raise RuntimeError(
                "No saved Go2 map found. Pass `map_dir:=/path/to/map_dir` or save a map first."
            )
        resolved = max(candidates, key=lambda path: path.stat().st_mtime)

    missing = [name for name in REQUIRED_MAP_FILES if not (resolved / name).exists()]
    if missing:
        raise RuntimeError(f"Invalid map_dir: {resolved}. Missing required files: {missing}")
    return resolved


def _build_nodes(context, *args, **kwargs):
    map_dir = _resolve_map_dir(
        LaunchConfiguration("map_dir").perform(context).strip(),
        LaunchConfiguration("map_root").perform(context).strip(),
    )
    go2_navigation_share = get_package_share_path("go2_navigation")
    global_planner_share = get_package_share_path("global_planner")
    config_file = go2_navigation_share / "config" / "go2_global_planner.yaml"
    rviz_config = global_planner_share / "rviz" / "path_planning_on_static_layer.rviz"

    map_file = map_dir / "map.pcd"
    ground_file = map_dir / "ground.pcd"

    pcl_publisher = Node(
        package="mcl_3dl",
        executable="pcl_publisher",
        output="screen",
        parameters=[
            str(config_file),
            {
                "map_dir": str(map_file),
                "ground_dir": str(ground_file),
            },
        ],
    )

    global_planner = Node(
        package="global_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[str(config_file)],
    )

    map_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="go2_global_planner_map_to_base_link",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("start_x"),
            "--y", LaunchConfiguration("start_y"),
            "--z", LaunchConfiguration("start_z"),
            "--roll", LaunchConfiguration("start_roll"),
            "--pitch", LaunchConfiguration("start_pitch"),
            "--yaw", LaunchConfiguration("start_yaw"),
            "--frame-id", "map",
            "--child-frame-id", "base_link",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_global_planner_rviz",
        output="screen",
        arguments=["-d", str(rviz_config)],
    )

    return [
        LogInfo(msg=["Using Go2 planner map_dir: ", str(map_dir)]),
        LogInfo(msg=["Static planner start pose in map frame: x=", LaunchConfiguration("start_x"),
                     " y=", LaunchConfiguration("start_y"),
                     " z=", LaunchConfiguration("start_z"),
                     " yaw=", LaunchConfiguration("start_yaw")]),
        pcl_publisher,
        global_planner,
        map_to_base,
        rviz,
    ]


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
            DeclareLaunchArgument("start_x", default_value="0.0"),
            DeclareLaunchArgument("start_y", default_value="0.0"),
            DeclareLaunchArgument("start_z", default_value="0.0"),
            DeclareLaunchArgument("start_roll", default_value="0.0"),
            DeclareLaunchArgument("start_pitch", default_value="0.0"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            OpaqueFunction(function=_build_nodes),
        ]
    )
