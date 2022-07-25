import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    display_launch_path = os.path.join(
        get_package_share_directory('simrobo_description'), 'launch', 'display.launch.py')
    gzserver_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    gzclient_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
    
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'simrobo',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.2',
                    '-topic', '/robot_description'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless.'),
            
        DeclareLaunchArgument(
            'server',
            default_value='true',
            description='Set to "false" not to run gzserver.'),
        
        # ===== display launch (RViz2) ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path)
        ),
        
        # ===== Gazebo ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_path),
            condition=IfCondition(LaunchConfiguration('server'))
        ),
        
        IncludeLaunchDescription(
        
            PythonLaunchDescriptionSource(gzclient_path),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        
        spawn_entity        
    ])
