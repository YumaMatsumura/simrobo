import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the directory
    display_launch_file = os.path.join(
        get_package_share_directory('simrobo_description'), 'launch', 'display.launch.py')
    gzserver_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    gzclient_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
    world_file_name = 'test_world.world'
    world_file = os.path.join(
        get_package_share_directory('simrobo_gazebo'), 'worlds', world_file_name)

    # Create the launch configuration variables
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    use_3d_lidar = LaunchConfiguration('use_3d_lidar')
    use_gpu = LaunchConfiguration('use_gpu')
    pose = {'x': LaunchConfiguration('x_pose', default='9.0'),
            'y': LaunchConfiguration('y_pose', default='-5.0'),
            'z': LaunchConfiguration('z_pose', default='0.2'),
            'R': LaunchConfiguration('roll', default='0.0'),
            'P': LaunchConfiguration('pitch', default='0.0'),
            'Y': LaunchConfiguration('yaw', default='3.14')}
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')
    declare_server_cmd = DeclareLaunchArgument(
        'server',
        default_value='true',
        description='Set to "false" not to run gzserver.')
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
        default_value='True',
        description='Whether to use rviz')
    declare_use_3d_lidar_cmd = DeclareLaunchArgument(
        'use_3d_lidar',
        default_value='False',
        description='Whether to use 3d_lidar')
    declare_use_gpu_cmd = DeclareLaunchArgument(
        'use_gpu',
        default_value='False',
        description='Whether to use Gazebo gpu_ray')

    # Specify the actions
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

    # Create nodes
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simrobo',
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-R', pose['R'],
            '-P', pose['P'],
            '-Y', pose['Y'],
            '-topic', '/robot_description'],
        output='screen')

    # Create execute process
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster',
             '--set-state', 'active'],
        output='screen')
    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'velocity_controller',
             '--set-state', 'active'],
        output='screen')

    return LaunchDescription([
        declare_gui_cmd,
        declare_server_cmd,
        declare_use_sim_time_cmd,
        declare_use_composition_cmd,
        declare_use_rviz_cmd,
        declare_use_3d_lidar_cmd,
        declare_use_gpu_cmd,

        # ===== display launch (RViz2) ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={
                'use_rviz': use_rviz,
                'use_ignition': 'false',
                'use_3d_lidar': use_3d_lidar,
                'use_gpu': use_gpu}.items()),

        # ===== Gazebo ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch_file),
            launch_arguments={'world': world_file}.items(),
            condition=IfCondition(server)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch_file),
            condition=IfCondition(gui)),
        spawn_entity,

        # ===== Driver Nodes ===== #
        bringup_composable_driver_nodes,
        bringup_driver_nodes,

        # ===== Ros2 Control ===== #
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[velocity_controller]
            )),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[velocity_controller]
            ))
    ])
