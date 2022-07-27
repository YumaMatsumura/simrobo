import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('simrobo_driver'), 'cfg')
    config_file = os.path.join(config_dir, 'f710.config.yaml')
    
    driver_node = ComposableNode(
        name='',
        package='simrobo_driver',
        plugin='simrobo_driver::Driver',
        parameters=[
            {'wheel_radius_size_m': 0.1},
            {'tread_width_m': 0.33},
            {'global_frame_id': 'odom'},
            {'base_frame_id': 'base_footprint'}
        ]
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='simrobo_container',
            namespace='driver',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                driver_node
            ],
            arguments=['--ros-args', '--log-level', 'INFO'],
            output='screen'
        ),
        
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            parameters=[{'device_name': '/dev/input/js0'}]
        ),
        
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[config_file]
        )
    
    ])
