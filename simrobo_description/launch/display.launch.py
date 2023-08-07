import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ignition = LaunchConfiguration('use_ignition')
    use_3d_lidar = LaunchConfiguration('use_3d_lidar')
    use_gpu = LaunchConfiguration('use_gpu')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to use rviz')
    declare_use_ignition_cmd = DeclareLaunchArgument(
        'use_ignition',
        default_value='True',
        description='Use ignition gazebo if true, use gazebo if false')
    declare_use_3d_lidar_cmd = DeclareLaunchArgument(
        'use_3d_lidar',
        default_value='False',
        description='Whether to use 3d_lidar')
    declare_use_gpu_cmd = DeclareLaunchArgument(
        'use_gpu',
        default_value='False',
        description='Whether to use Gazebo gpu_ray')
    
    
    # Get the rviz directory
    rviz_file = os.path.join(get_package_share_directory('simrobo_description'), 'rviz', 'simrobo.rviz')
    xacro_file = os.path.join(get_package_share_directory('simrobo_description'), 'urdf', 'simrobo.urdf.xacro')
    
    robot_description = Command(['xacro', ' ', xacro_file, ' use_ignition:=', use_ignition, ' use_3d_lidar:=', use_3d_lidar, ' gpu:=', use_gpu])
    
        
    # Create nodes
    bringup_display_nodes = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}]),
            
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
            
        Node(
            condition=IfCondition(use_rviz),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),
            
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}])
            
        ])


    return LaunchDescription([
        declare_use_sim_time_cmd,
        
        declare_use_rviz_cmd,
        
        declare_use_ignition_cmd,
        
        declare_use_3d_lidar_cmd,
        
        declare_use_gpu_cmd,

        bringup_display_nodes
    ])
