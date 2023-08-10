import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch directory
    simrobo_bringup_dir = get_package_share_directory('simrobo_bringup')
    joy_params_file = os.path.join(simrobo_bringup_dir, 'params', 'joy_params.yaml')

    # Create the launch configuration variables
    use_keyboard = LaunchConfiguration('use_keyboard')
    declare_use_keyboard_cmd = DeclareLaunchArgument(
        'use_keyboard',
        default_value='False',
        description='Whether to use keyboard to control robot')

    # Create nodes
    load_joy_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_keyboard])),
        actions=[
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                parameters=[{'device_name': '/dev/input/js0'}]),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                parameters=[joy_params_file])
            ])

    load_keyboard_node = Node(
        condition=IfCondition(use_keyboard),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e')

    return LaunchDescription([
        declare_use_keyboard_cmd,
        load_joy_nodes,
        load_keyboard_node
    ])
