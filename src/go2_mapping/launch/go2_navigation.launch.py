"""go2_navigation.launch.py
Autonomous navigation for Go2 using a previously saved LeGO-LOAM pose graph.

Equivalent to:
    ros2 launch p2p_move_base p2p_move_base_localization.launch

But wired for real Go2 hardware (utlidar + mid360 + aurora odometry).

Usage:
    ros2 launch go2_mapping go2_navigation.launch.py

    or:

    ros2 launch go2_mapping go2_navigation.launch.py \\
        map_dir:=/root/dddmr_navigation/maps/go2/2026-06-25-12-00-00

Map arguments:
    map_dir   -- full path to the pose-graph directory produced by
                 save_go2_map.py. If empty, the latest valid map under
                 map_root is used.
    map_root  -- directory containing timestamped saved maps.

Optional arguments (all have sensible defaults):
    interface        -- network interface for Aurora (default: enp88s0)
    aurora_ip        -- Aurora SLAM device IP (default: 192.168.11.1)
    use_go2          -- bring up Go2 base driver (default: true)
    use_aurora       -- bring up Aurora SLAM node (default: true)
    use_mid360       -- bring up Livox Mid-360 driver (default: true)
    use_go2_camera   -- bring up Go2 camera (default: true)
    raw_lidar_topic  -- raw merged cloud input (default: /merged/points)
    lidar_topic      -- transformed cloud for MCL (default: /go2/lidar_points_base)
    odom_topic       -- odometry topic (default: /slamware_ros_sdk_server_node/odom)
    rviz             -- launch RViz (default: true)
"""

from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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


REQUIRED_MAP_FILES = ("poses.pcd", "edges.pcd", "map.pcd", "ground.pcd")


def _is_pose_graph_dir(path: Path):
    return path.is_dir() and all((path / name).exists() for name in REQUIRED_MAP_FILES)


def _latest_pose_graph(root: Path):
    if not root.exists():
        return None
    candidates = [child for child in root.iterdir() if _is_pose_graph_dir(child)]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _resolve_map_dir(context):
    map_dir = LaunchConfiguration("map_dir").perform(context).strip()
    if map_dir:
        resolved = Path(map_dir).expanduser()
    else:
        map_root = Path(LaunchConfiguration("map_root").perform(context)).expanduser()
        latest = _latest_pose_graph(map_root)
        if latest is None:
            raise RuntimeError(
                "No saved Go2 pose graph found. Save a map first with "
                "`ros2 run go2_mapping save_go2_map.py`, or pass "
                "`map_dir:=/path/to/saved/map`."
            )
        resolved = latest

    if not _is_pose_graph_dir(resolved):
        missing = [name for name in REQUIRED_MAP_FILES if not (resolved / name).exists()]
        raise RuntimeError(
            f"Invalid map_dir: {resolved}. Missing required files: {missing}"
        )
    return resolved


def _build_navigation_nodes(context, *args, **kwargs):
    """OpaqueFunction: resolve map_dir at launch time and inject into config."""

    map_dir = _resolve_map_dir(context)
    lidar_topic = LaunchConfiguration("lidar_topic").perform(context)
    raw_lidar_topic = LaunchConfiguration("raw_lidar_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)

    go2_mapping_share = get_package_share_path("go2_mapping")
    base_config_file = go2_mapping_share / "config" / "go2_navigation.yaml"

    # ── Patch sub_maps.pose_graph_dir into a temp config ──────────────
    with base_config_file.open("r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    config.setdefault("sub_maps", {}).setdefault("ros__parameters", {})
    config["sub_maps"]["ros__parameters"]["pose_graph_dir"] = str(map_dir)
    config.setdefault("local_planner", {}).setdefault("ros__parameters", {})
    config["local_planner"]["ros__parameters"]["odom_topic"] = odom_topic

    temp_config = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="go2_navigation_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with temp_config as fh:
        yaml.safe_dump(config, fh, sort_keys=False)
    temp_config_path = temp_config.name

    # ── Feature extraction node (MCL variant of lego_loam imageProjection) ──
    mcl_feature = Node(
        package="lego_loam_bor",
        executable="mcl_feature",
        output="screen",
        parameters=[
            temp_config_path,
            {"use_sim_time": False},
        ],
        remappings=[
            ("lslidar_point_cloud", lidar_topic),
            ("odom", odom_topic),
        ],
    )

    # ── MCL 3DL: particle-filter localization.
    # The mcl_3dl executable creates both the "mcl_3dl" and "sub_maps" nodes.
    mcl_3dl = Node(
        package="mcl_3dl",
        executable="mcl_3dl",
        output="screen",
        parameters=[
            temp_config_path,
            {"use_sim_time": False},
        ],
        remappings=[
            ("odom", odom_topic),
        ],
    )

    # ── Pointcloud frame relay (same as mapping) ───────────────────────
    cloud_transform = Node(
        package="go2_mapping",
        executable="pointcloud_frame_relay.py",
        name="go2_nav_cloud_transform",
        output="screen",
        parameters=[
            {
                "input_topic": raw_lidar_topic,
                "output_topic": lidar_topic,
                "output_frame": "base_footprint",
                "transform_points": True,
            }
        ],
    )

    # ── Pointcloud merger (same as mapping) ────────────────────────────
    pointcloud_merger = Node(
        package="go2_mapping",
        executable="pointcloud_merger.py",
        name="go2_nav_pointcloud_merger",
        output="screen",
        parameters=[
            {
                "input_topics": ["/go2/lidar_points", "/livox/lidar"],
                "output_topic": "/merged/points",
                "output_frame": "base_footprint",
                "publish_rate": 10.0,
                "max_cloud_age": 0.2,
                "max_stamp_skew": 0.25,
                "odom_topic": odom_topic,
                "output_stamp_source": "odom",
                "sync_by_arrival_time": True,
            }
        ],
    )

    # ── Global planner ────────────────────────────────────────────────
    global_planner = Node(
        package="global_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[
            temp_config_path,
            {"use_sim_time": False},
        ],
    )

    # ── p2p_move_base ─────────────────────────────────────────────────
    p2p_move_base = Node(
        package="p2p_move_base",
        executable="p2p_move_base_node",
        output="screen",
        parameters=[
            temp_config_path,
            {"use_sim_time": False},
        ],
        remappings=[
            ("odom", odom_topic),
        ],
    )

    # ── Clicked-point → goal relay ────────────────────────────────────
    clicked2goal = Node(
        package="p2p_move_base",
        executable="clicked2goal.py",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return [
        LogInfo(msg=["Using Go2 map_dir: ", str(map_dir)]),
        cloud_transform,
        pointcloud_merger,
        mcl_feature,
        mcl_3dl,
        global_planner,
        p2p_move_base,
        clicked2goal,
    ]


def generate_launch_description():
    go2_mapping_share = get_package_share_path("go2_mapping")

    # ── Sensor bringup (reused from mapping launch) ────────────────────
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

    # ── RViz ─────────────────────────────────────────────────────────
    rviz_config = str(go2_mapping_share / "rviz" / "go2_navigation.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_navigation_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            # ── Env ─────────────────────────────────────────────────
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),

            # ── Required arguments ───────────────────────────────────
            DeclareLaunchArgument(
                "map_dir",
                default_value="",
                description=(
                    "Full path to the saved pose-graph directory. "
                    "If empty, the latest valid map under map_root is used "
                    "(e.g. /root/dddmr_navigation/maps/go2/2025-01-01-12-00-00). "
                    "Run `ros2 run go2_mapping save_go2_map.py` while mapping "
                    "to create this directory."
                ),
            ),
            DeclareLaunchArgument(
                "map_root",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "dddmr_navigation", "maps", "go2"]
                ),
                description=(
                    "Root directory containing timestamped Go2 maps. Used only "
                    "when map_dir is empty."
                ),
            ),

            # ── Optional hardware arguments ──────────────────────────
            DeclareLaunchArgument("interface", default_value="enp88s0"),
            DeclareLaunchArgument("aurora_ip", default_value="192.168.11.1"),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),

            # ── Topic remapping ──────────────────────────────────────
            DeclareLaunchArgument(
                "raw_lidar_topic", default_value="/merged/points"
            ),
            DeclareLaunchArgument(
                "lidar_topic", default_value="/go2/lidar_points_base"
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/slamware_ros_sdk_server_node/odom",
            ),

            # ── Bringup + Navigation nodes ───────────────────────────
            sensors_bringup,
            OpaqueFunction(function=_build_navigation_nodes),
            rviz,
        ]
    )
