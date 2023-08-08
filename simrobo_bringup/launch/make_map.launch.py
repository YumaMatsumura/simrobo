import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory
    bringup_dir = get_package_share_directory('simrobo_bringup')
    params_file = os.path.join(bringup_dir, 'params', 'make_map.yaml')
    teleop_twist_launch_file = os.path.join(bringup_dir, 'launch', 'teleop_twist_nodes.launch.py')
    rviz_file = os.path.join(bringup_dir, 'rviz', 'make_map.rviz')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Create nodes
    bringup_make_map_nodes = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_twist_launch_file)),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}])
    ])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        bringup_make_map_nodes
    ])
