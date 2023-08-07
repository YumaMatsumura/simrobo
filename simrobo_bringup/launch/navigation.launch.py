import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    simrobo_bringup_dir = get_package_share_directory('simrobo_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    rviz_file = os.path.join(simrobo_bringup_dir, 'rviz',  'navigation.rviz')

    # Create the launch configuration variables
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(simrobo_bringup_dir, 'maps', 'sample_map', 'sample_map.yaml'),
        description='Full path to map yaml file to load')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(simrobo_bringup_dir, 'params', 'navigation_params.yaml'),
        description='Full path to the ROS2 parameters file to use for cleaning nodes')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    # Specify the actions
    bringup_navigation_nodes = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'use_composition': use_composition}.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ])

    return LaunchDescription([
        declare_map_file_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_use_composition_cmd,
        bringup_navigation_nodes
    ])
