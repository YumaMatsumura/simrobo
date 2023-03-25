import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node


def get_map_folder_path(base_path, map_name):
    return [base_path, map_name]
    
def get_map_file_path(base_path, map_name):
    return [base_path, map_name, map_name]

def generate_launch_description():
    # Create the launch configuration variables
    map_name = LaunchConfiguration('map_name')
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='/sample_map',
        description='Set map name')
    
    # Declare base path
    maps_folder_path = os.environ["MAPS_FOLDER_PATH"]
    
    
    return LaunchDescription([
        declare_map_name_cmd,
        
        ExecuteProcess(
            cmd=[
                'mkdir -p',
                get_map_folder_path(maps_folder_path, map_name)
                ],
            shell=True),
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_saver_cli',
                    name='map_saver_cli',
                    output='screen',
                    arguments=['-f', get_map_file_path(maps_folder_path, map_name)],
                    parameters=[{'save_map_timeout': 10000.0}])
                ]
            )
    ])
