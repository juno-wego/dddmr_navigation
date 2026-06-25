from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Declare launch arguments
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='enp88s0',
        description='Network interface for Unitree SDK'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable RViz and joint_state_publisher_gui'
    )

    # Get paths
    go2_base_path = get_package_share_path('go2_base')
    go2_description_path = get_package_share_path('go2_description')

    # Include go2_description's display_launch.py with gui parameter
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(go2_description_path / 'launch/display_launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    # Run go2_base driver node with interface parameter
    go2_driver_node = Node(
        package='go2_base',
        executable='go2_driver',
        name='go2_driver',
        # remappings=[('/cmd_vel', '/go2/cmd_vel')],
        output='screen',
        arguments=[LaunchConfiguration('interface')]
    )

    go2_camera_node =Node(
        package="go2_base",
        executable="go2_camera_publisher",
        name="go2_camera_publisher"
    )

    return LaunchDescription([
        interface_arg,
        gui_arg,
        display_launch,
        go2_driver_node,
        go2_camera_node
    ])
