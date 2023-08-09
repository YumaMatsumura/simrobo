import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the directory
    ros_gz_dir = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_file = os.path.join(ros_gz_dir, 'launch', 'gz_sim.launch.py')
    description_dir = get_package_share_directory('simrobo_description')
    display_launch_file = os.path.join(description_dir, 'launch', 'display.launch.py')
    simrobo_ignition_dir = get_package_share_directory('simrobo_ignition')
    gui_config_file = os.path.join(simrobo_ignition_dir, 'gui', 'gui.config')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed driver node')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to use rviz')

    # Create nodes
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
                   '/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/depth_rect_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen')

    bringup_composable_driver_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            Node(
                package='rclcpp_components',
                executable='component_container',
                name='simrobo_container',
                output='screen'),

            LoadComposableNodes(
                target_container='simrobo_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='simrobo_driver',
                        plugin='simrobo_driver::Driver',
                        name='simrobo_driver_node',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'wheel_radius_size_m': 0.1},
                            {'tread_width_m': 0.33},
                            {'odom_frame_id': 'odom'},
                            {'base_frame_id': 'base_footprint'}
                            ])
                    ]
                )
            ]
        )

    bringup_driver_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='simrobo_driver',
                executable='driver',
                name='simrobo_driver_node',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'wheel_radius_size_m': 0.1},
                    {'tread_width_m': 0.33},
                    {'odom_frame_id': 'odom'},
                    {'base_frame_id': 'base_footprint'}
                    ]
                )
            ]
        )

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
        declare_use_sim_time_cmd,
        declare_use_composition_cmd,
        declare_use_rviz_cmd,

        # ===== Display Launch (RViz 2) ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
                'use_ignition': 'true',
                'use_3d_lidar': 'false',
                'use_gpu': 'false'}.items()),

        # ===== Ignition ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch_file),
            launch_arguments=[
                ('gz_args', [' -r -v 3 empty.sdf' + ' --gui-config ' + gui_config_file])
                ]),
        gz_spawn_entity,
        gz_bridge,

        # ===== Driver Nodes ===== #
        bringup_composable_driver_nodes,
        bringup_driver_nodes,

        # ===== ROS 2 Control ===== #
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_velocity_controller]))
        ])
