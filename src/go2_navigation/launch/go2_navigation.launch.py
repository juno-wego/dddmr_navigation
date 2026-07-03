"""go2_navigation.launch.py
Autonomous navigation for Go2 using a previously saved LeGO-LOAM pose graph.

Equivalent to:
    ros2 launch p2p_move_base p2p_move_base_localization.launch

But wired for real Go2 hardware (Go2 + Mid-360 + optional Aurora).

Usage:
    ros2 launch go2_navigation go2_navigation.launch.py

    or:

    ros2 launch go2_navigation go2_navigation.launch.py \\
        map_dir:=/root/dddmr_navigation/maps/go2/2026-06-25-12-00-00

Map arguments:
    map_dir   -- full path to the pose-graph directory produced by
                 save_go2_map.py. If empty, the latest valid map under
                 map_root is used.
    map_root  -- directory containing timestamped saved maps.

Optional arguments (all have sensible defaults):
    interface        -- network interface for Go2 SDK (default: enp88s0)
    aurora_ip        -- Aurora SLAM device IP (default: 192.168.11.1)
    odom_source      -- odometry source for MCL: aurora or go2 (default: go2)
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
ODOM_PROFILES = {
    "aurora": {
        "odom_topic": "/slamware_ros_sdk_server_node/odom",
        "odom_frame": "aurora_odom",
    },
    "go2": {
        "odom_topic": "/odom",
        "odom_frame": "odom",
    },
}
DIRECT_SENSOR_PROFILES = {
    "utlidar": {
        "point_topic": "/go2/lidar_points",
        "laser": {
            "num_vertical_scans": 16,
            "num_horizontal_scans": 440,
            "vertical_angle_bottom": -15.0,
            "vertical_angle_top": 15.0,
            "scan_period": 0.1,
        },
        "imageProjection": {
            "segment_valid_point_num": 5,
            "segment_valid_line_num": 2,
            "segment_theta": 60.0,
            "minimum_detection_range": 0.5,
            "maximum_detection_range": 100.0,
            "distance_for_patch_between_rings": 1.0,
            "stitcher_num": 0,
            "trt_model_path": "",
            "projected_image_stack_size": 1,
            "ground_fov_bottom": -0.2617994,
            "ground_fov_top": -0.087266,
            "ground_positive_start": 0.0,
            "ground_positive_stop": 3.1415926,
            "ground_negative_start": 0.0,
            "ground_negative_stop": -3.1415926,
        },
    },
    "livox": {
        "point_topic": "/livox/lidar",
        "laser": {
            "num_vertical_scans": 32,
            "num_horizontal_scans": 1000,
            "vertical_angle_bottom": -7.0,
            "vertical_angle_top": 52.0,
            "scan_period": 0.1,
        },
        "imageProjection": {
            "segment_valid_point_num": 5,
            "segment_valid_line_num": 2,
            "segment_theta": 60.0,
            "minimum_detection_range": 0.5,
            "maximum_detection_range": 100.0,
            "distance_for_patch_between_rings": 1.0,
            "stitcher_num": 3,
            "trt_model_path": "",
            "projected_image_stack_size": 1,
            "ground_fov_bottom": -0.12217,
            "ground_fov_top": 0.14,
            "ground_positive_start": 0.0,
            "ground_positive_stop": 1.57,
            "ground_negative_start": 0.0,
            "ground_negative_stop": -1.57,
        },
    },
}
LOCALIZATION_MCL_OVERRIDES = {
    # Run measurement updates for smaller motions so in-place rotation
    # gets corrected before drift accumulates.
    "update_min_d": 0.03,
    # Trigger measurement updates earlier during in-place turns.
    "update_min_a": 0.008,
    # Reduce map->odom smoothing so map correction catches rotation drift faster.
    "lpf_step": 0.35,
    "jump_dist": 0.5,
    "jump_ang": 0.18,
    # Explore yaw space more aggressively when the estimate starts to drift.
    "resample_var_yaw": 0.40,
    "expansion_var_x": 0.7,
    "expansion_var_y": 0.7,
    "expansion_var_yaw": 0.9,
    # Trust rotational odometry less so the filter can snap back to the map.
    "odom_err_lin_ang": 0.5,
    "odom_err_ang_lin": 0.9,
    "odom_err_ang_ang": 1.8,
    # Relax the bias against larger corrections from the map.
    "bias_var_dist": 3.0,
    "bias_var_ang": 6.28,
    # Re-expand particles when match quality drops during turns.
    "match_ratio_thresh": 0.10,
}
LOCALIZATION_LIKELIHOOD_OVERRIDES = {
    # Restore the tighter default used by the stable MCL demos.
    "match_dist_min": 0.30,
}


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


def _configure_direct_sensor_profile(config, lidar_source):
    profile = DIRECT_SENSOR_PROFILES[lidar_source]
    params = config.setdefault("mcl_ip", {}).setdefault("ros__parameters", {})
    laser = params.setdefault("laser", {})
    image = params.setdefault("imageProjection", {})

    laser.update(profile["laser"])
    image.update(profile["imageProjection"])

    # Merged clouds lose sensor ring/line structure, but direct raw sensor topics keep it.
    # Remove merged-cloud-only overrides so localization behaves like the stable bag demos.
    for key in (
        "unorganized_cloud_mode",
        "unorganized_ground_ring_count",
        "unorganized_ground_min_z",
        "unorganized_ground_max_z",
        "unorganized_ground_max_radius",
        "unorganized_ground_edge_stride",
    ):
        image.pop(key, None)

    return profile["point_topic"]


def _configure_odom_profile(config, odom_source):
    profile = ODOM_PROFILES[odom_source]
    mcl_params = config.setdefault("mcl_3dl", {}).setdefault("ros__parameters", {})
    local_planner_params = config.setdefault("local_planner", {}).setdefault("ros__parameters", {})
    mcl_params["odom_frame"] = profile["odom_frame"]
    local_planner_params["odom_topic"] = profile["odom_topic"]
    return profile["odom_topic"]


def _configure_localization_tuning(config):
    mcl_params = config.setdefault("mcl_3dl", {}).setdefault("ros__parameters", {})
    mcl_params.update(LOCALIZATION_MCL_OVERRIDES)
    likelihood = mcl_params.setdefault("likelihood", {})
    likelihood.update(LOCALIZATION_LIKELIHOOD_OVERRIDES)


def _configure_no_entry_zones(config, context):
    perception_3d_share = get_package_share_path("perception_3d")
    no_entry_pcd_arg = LaunchConfiguration("no_entry_pcd").perform(context).strip()
    no_entry_pcd = (
        Path(no_entry_pcd_arg).expanduser()
        if no_entry_pcd_arg
        else perception_3d_share / "map" / "no_entry1.pcd"
    )

    if not no_entry_pcd.exists():
        raise RuntimeError(
            f"Default no-entry PCD does not exist: {no_entry_pcd}"
        )

    no_entry_zone_config = {
        "no_entry_layer": {
            "ros__parameters": {
                "zone1": {
                    "pcd": str(no_entry_pcd),
                    "is_enabled": True,
                }
            }
        }
    }
    temp_no_entry_config = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="go2_no_entry_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with temp_no_entry_config as fh:
        yaml.safe_dump(no_entry_zone_config, fh, sort_keys=False)
    temp_no_entry_config_path = temp_no_entry_config.name

    for section in ("perception_3d_local", "perception_3d_global"):
        params = config.setdefault(section, {}).setdefault("ros__parameters", {})
        plugins = params.setdefault("plugins", [])
        if "no_entry_layer" not in plugins:
            plugins.insert(1 if plugins else 0, "no_entry_layer")
        no_entry_layer = params.setdefault("no_entry_layer", {})
        no_entry_layer["plugin"] = "perception_3d::NoEntryLayer"
        no_entry_layer["no_entry_zone_pcd_file_dir"] = temp_no_entry_config_path
        no_entry_layer.setdefault(
            "inflation_distance",
            1.0 if section == "perception_3d_global" else 0.9,
        )

    return temp_no_entry_config_path


def _build_navigation_nodes(context, *args, **kwargs):
    """OpaqueFunction: resolve map_dir at launch time and inject into config."""

    map_dir = _resolve_map_dir(context)
    odom_source = LaunchConfiguration("odom_source").perform(context)
    lidar_source = LaunchConfiguration("lidar_source").perform(context)
    lidar_topic = LaunchConfiguration("lidar_topic").perform(context)
    raw_lidar_topic = LaunchConfiguration("raw_lidar_topic").perform(context)

    go2_navigation_share = get_package_share_path("go2_navigation")
    base_config_file = go2_navigation_share / "config" / "go2_navigation.yaml"

    # ── Patch sub_maps.pose_graph_dir into a temp config ──────────────
    with base_config_file.open("r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    config.setdefault("sub_maps", {}).setdefault("ros__parameters", {})
    config["sub_maps"]["ros__parameters"]["pose_graph_dir"] = str(map_dir)

    if odom_source not in ODOM_PROFILES:
        raise RuntimeError(
            f"Unsupported odom_source: {odom_source}. Choose one of: aurora, go2."
        )
    odom_topic = _configure_odom_profile(config, odom_source)
    _configure_localization_tuning(config)
    no_entry_zone_config_path = _configure_no_entry_zones(config, context)

    if lidar_source == "merged":
        effective_lidar_topic = lidar_topic
    elif lidar_source in DIRECT_SENSOR_PROFILES:
        effective_lidar_topic = _configure_direct_sensor_profile(config, lidar_source)
    else:
        raise RuntimeError(
            f"Unsupported lidar_source: {lidar_source}. "
            "Choose one of: merged, utlidar, livox."
        )

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
            ("lslidar_point_cloud", effective_lidar_topic),
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

    sensor_pipeline_nodes = []
    if lidar_source == "merged":
        # ── Pointcloud frame relay (same as mapping) ───────────────────
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

        # ── Pointcloud merger (same as mapping) ────────────────────────
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
        sensor_pipeline_nodes.extend([cloud_transform, pointcloud_merger])

    # ── Global planner ────────────────────────────────────────────────
    global_planner = Node(
        package="global_planner",
        executable="global_planner_node",
        output="screen",
        parameters=[
            temp_config_path,
            {"use_sim_time": False},
        ],
        condition=IfCondition(LaunchConfiguration("start_navigation")),
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
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    # ── Clicked-point → goal relay ────────────────────────────────────
    clicked2goal = Node(
        package="p2p_move_base",
        executable="clicked2goal.py",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    return [
        LogInfo(msg=["Using Go2 map_dir: ", str(map_dir)]),
        LogInfo(msg=["Using odom_source: ", odom_source, " -> ", odom_topic]),
        LogInfo(msg=["Using lidar_source: ", lidar_source, " -> ", effective_lidar_topic]),
        LogInfo(msg=["Using no_entry zones: ", no_entry_zone_config_path]),
        *sensor_pipeline_nodes,
        mcl_feature,
        mcl_3dl,
        global_planner,
        p2p_move_base,
        clicked2goal,
    ]


def generate_launch_description():
    go2_navigation_share = get_package_share_path("go2_navigation")

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
            "odom_source": LaunchConfiguration("odom_source"),
            "use_go2": LaunchConfiguration("use_go2"),
            "use_aurora": LaunchConfiguration("use_aurora"),
            "use_mid360": LaunchConfiguration("use_mid360"),
            "use_go2_camera": LaunchConfiguration("use_go2_camera"),
            "lidar_frame_id": "utlidar",
            "show_rviz": "false",
        }.items(),
    )

    # ── RViz ─────────────────────────────────────────────────────────
    rviz_config = str(go2_navigation_share / "rviz" / "go2_navigation.rviz")
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
            DeclareLaunchArgument(
                "odom_source",
                default_value="go2",
                choices=["aurora", "go2"],
                description=(
                    "Odometry source for localization. "
                    "Use 'go2' for Go2 internal IMU/odometry, or 'aurora' for Aurora odometry."
                ),
            ),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_navigation",
                default_value="true",
                description=(
                    "Start planner/controller nodes together with localization. "
                    "Set false to run a stable localization-only session first."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_source",
                default_value="livox",
                description=(
                    "Localization point-cloud source. "
                    "Use 'livox' or 'utlidar' for stable direct-sensor MCL, "
                    "or 'merged' for the existing merged-cloud path."
                ),
            ),

            # ── Topic remapping ──────────────────────────────────────
            DeclareLaunchArgument(
                "raw_lidar_topic", default_value="/merged/points"
            ),
            DeclareLaunchArgument(
                "lidar_topic", default_value="/go2/lidar_points_base"
            ),
            DeclareLaunchArgument(
                "no_entry_pcd",
                default_value="",
                description=(
                    "Absolute path to a keepout/no-entry zone PCD. "
                    "If empty, use perception_3d's bundled sample no_entry1.pcd."
                ),
            ),
            # ── Bringup + Navigation nodes ───────────────────────────
            sensors_bringup,
            OpaqueFunction(function=_build_navigation_nodes),
            rviz,
        ]
    )
