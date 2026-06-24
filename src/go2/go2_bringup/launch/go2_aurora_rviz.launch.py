from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from pathlib import Path


def make_rviz_node(context, *args, **kwargs):
    if LaunchConfiguration("show_rviz").perform(context).lower() != "true":
        return []

    odom_source = LaunchConfiguration("odom_source").perform(context)
    use_aurora = LaunchConfiguration("use_aurora").perform(context).lower() == "true"
    use_go2 = LaunchConfiguration("use_go2").perform(context).lower() == "true"
    share = get_package_share_path("go2_bringup")
    rviz_template = share / "rviz" / "go2_aurora_sensors.rviz"
    rviz_config = Path(f"/tmp/go2_aurora_sensors_{odom_source}.rviz")

    text = rviz_template.read_text()
    if odom_source == "aurora" and use_aurora:
        fixed_frame = "aurora_odom"
    elif odom_source == "go2" and use_go2:
        fixed_frame = "odom"
    else:
        fixed_frame = "base_footprint"

    text = text.replace("Fixed Frame: aurora_odom", f"Fixed Frame: {fixed_frame}")

    if odom_source == "aurora":
        text = text.replace("Value: /scan", "Value: /slamware_ros_sdk_server_node/scan")
        text = text.replace("Value: /slamware_map", "Value: /slamware_ros_sdk_server_node/map")
        text = text.replace("Value: /odom", "Value: /slamware_ros_sdk_server_node/odom")

    if odom_source == "go2":
        text = text.replace("Value: /scan", "Value: /aurora/slamware_ros_sdk_server_node/scan")
        text = text.replace("Value: /slamware_map", "Value: /aurora/slamware_ros_sdk_server_node/map")
        text = text.replace("Name: Aurora Odom", "Name: Go2 Odom")

    rviz_config.write_text(text)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="go2_aurora_rviz",
            output="screen",
            arguments=["-d", str(rviz_config)],
        )
    ]


def generate_launch_description():
    interface = LaunchConfiguration("interface")
    aurora_ip = LaunchConfiguration("aurora_ip")
    odom_source = LaunchConfiguration("odom_source")
    use_go2 = LaunchConfiguration("use_go2")
    use_aurora = LaunchConfiguration("use_aurora")
    use_mid360 = LaunchConfiguration("use_mid360")
    use_go2_camera = LaunchConfiguration("use_go2_camera")

    use_aurora_odom = IfCondition(
        PythonExpression(["'", use_aurora, "' == 'true' and '", odom_source, "' == 'aurora'"])
    )
    use_go2_odom = IfCondition(
        PythonExpression(["'", use_go2, "' == 'true' and '", odom_source, "' == 'go2'"])
    )
    use_aurora_namespaced = IfCondition(
        PythonExpression(["'", use_aurora, "' == 'true' and '", odom_source, "' == 'go2'"])
    )

    robot_description = (
        get_package_share_path("go2_description") / "urdf" / "go2_description.urdf"
    ).read_text()

    aurora_launch_for_aurora_odom = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slamware_ros_sdk"),
                    "launch",
                    "slamware_ros_sdk_server_node.xml",
                ]
            )
        ),
        launch_arguments={"ip_address": aurora_ip}.items(),
        condition=use_aurora_odom,
    )

    aurora_launch_for_go2_odom = GroupAction(
        [
            PushRosNamespace("aurora"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slamware_ros_sdk"),
                            "launch",
                            "slamware_ros_sdk_server_node.xml",
                        ]
                    )
                ),
                launch_arguments={"ip_address": aurora_ip}.items(),
            ),
        ],
        condition=use_aurora_namespaced,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    go2_driver_with_aurora_odom = Node(
        package="go2_base",
        executable="go2_driver",
        name="go2_driver",
        output="screen",
        arguments=[interface],
        remappings=[
            ("odom", "/go2/wheel_odom"),
            ("/tf", "/go2/tf"),
        ],
        condition=IfCondition(
            PythonExpression(["'", use_go2, "' == 'true' and '", odom_source, "' == 'aurora'"])
        ),
    )

    go2_driver_with_go2_odom = Node(
        package="go2_base",
        executable="go2_driver",
        name="go2_driver",
        output="screen",
        arguments=[interface],
        condition=use_go2_odom,
    )

    go2_camera = Node(
        package="go2_base",
        executable="go2_camera_publisher",
        name="go2_camera_publisher",
        output="screen",
        parameters=[
            {
                "frame_id": "front_camera",
                "topic_base": "/go2/camera",
                "camera_name": "go2_camera",
                "camera_info_url": ParameterValue(
                    [
                        "file://",
                        PathJoinSubstitution(
                            [FindPackageShare("go2_base"), "config", "go2_camera_info.yaml"]
                        ),
                    ],
                    value_type=str,
                ),
            }
        ],
        condition=IfCondition(use_go2_camera),
    )

    mid360_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"lvx_file_path": "/tmp/livox_test.lvx"},
            {
                "user_config_path": ParameterValue(
                    PathJoinSubstitution(
                        [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
                    ),
                    value_type=str,
                )
            },
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
        condition=IfCondition(use_mid360),
    )

    aurora_base_to_go2_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aurora_base_to_go2_base_footprint",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "aurora_base",
            "--child-frame-id", "base_footprint",
        ],
        condition=IfCondition(
            PythonExpression(["'", use_aurora, "' == 'true' and '", odom_source, "' == 'aurora'"])
        ),
    )

    base_link_to_mid360 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_mid360",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "livox_frame",
        ],
        condition=IfCondition(use_mid360),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="enp88s0",
                description="Network interface used by Unitree Go2 SDK.",
            ),
            DeclareLaunchArgument(
                "aurora_ip",
                default_value="192.168.11.1",
                description="Aurora device IP address.",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="aurora",
                choices=["aurora", "go2"],
                description="Select /odom owner: aurora uses Aurora odometry, go2 uses Go2 internal odometry.",
            ),
            DeclareLaunchArgument("use_go2", default_value="true"),
            DeclareLaunchArgument("use_aurora", default_value="true"),
            DeclareLaunchArgument("use_mid360", default_value="true"),
            DeclareLaunchArgument("use_go2_camera", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            aurora_launch_for_aurora_odom,
            aurora_launch_for_go2_odom,
            robot_state_publisher,
            go2_driver_with_aurora_odom,
            go2_driver_with_go2_odom,
            go2_camera,
            mid360_driver,
            aurora_base_to_go2_base_footprint,
            base_link_to_mid360,
            OpaqueFunction(function=make_rviz_node),
        ]
    )
