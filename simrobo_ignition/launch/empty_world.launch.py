import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory
    ros_gz_dir = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_file = os.path.join(ros_gz_dir, 'launch', 'gz_sim.launch.py')
    description_dir = get_package_share_directory('simrobo_description')
    display_launch_file = os.path.join(description_dir, 'launch', 'display.launch.py')
    simrobo_ignition_dir = get_package_share_directory('simrobo_ignition')
    gui_config_file = os.path.join(simrobo_ignition_dir, 'gui', 'gui.config')

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'simrobo',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                   '-allow_renaming', 'true'])

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/depth_rect_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster',
             '--set-state', 'active'],
        shell=True,
        output='screen')

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'velocity_controller',
             '--set-state', 'active'],
        shell=True,
        output='screen')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
                'use_ignition': 'true',
                'use_3d_lidar': 'false',
                'use_gpu': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch_file),
            launch_arguments=[('gz_args', [' -r -v 3 empty.sdf'
                                            + ' --gui-config ' + gui_config_file
                                            ])]),
        gz_spawn_entity,
        gz_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_velocity_controller]))
        ])
