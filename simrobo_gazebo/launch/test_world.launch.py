import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
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
    use_rviz = LaunchConfiguration('use_rviz')
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
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to use rviz')
    
    # Create nodes
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'simrobo',
                        '-x', pose['x'],
                        '-y', pose['y'],
                        '-z', pose['z'],
                        '-R', pose['R'],
                        '-P', pose['P'],
                        '-Y', pose['Y'],
                    '-topic', '/robot_description'],
        output='screen'
    )
    
    components_node = Node(
        package='rclcpp_components',
        executable='component_container',
        name='simrobo_container',
        output='screen')
    
    load_driver_node = LoadComposableNodes(
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
                    {'base_frame_id': 'base_footprint'}])
        ])
    
    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "joint_state_broadcaster", 
             "--set-state", "active"],
        output="screen"
    )
    
    velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "velocity_controller", 
             "--set-state", "active"],
        output="screen"
    )

    return LaunchDescription([
        declare_gui_cmd,
        
        declare_server_cmd,
        
        declare_use_sim_time_cmd,
        
        declare_use_rviz_cmd,
        
        # ===== display launch (RViz2) ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),
        
        # ===== Gazebo ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch_file),
            launch_arguments={'world': world_file}.items(),
            condition=IfCondition(server)
        ),
        
        IncludeLaunchDescription(
        
            PythonLaunchDescriptionSource(gzclient_launch_file),
            condition=IfCondition(gui)
        ),
        
        spawn_entity,
        
        # ===== Ros2 Control ===== #        
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[velocity_controller]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[velocity_controller]
            )
        ),      
        
        # ===== Components Nodes ===== #
        components_node,
        
        load_driver_node
    ])
