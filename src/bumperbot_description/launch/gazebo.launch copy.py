import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package and file paths
    pkg_share = FindPackageShare(package='bumperbot_description').find('bumperbot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/bumperbot.urdf.xacro')
    controller_params_file = os.path.join(
        FindPackageShare(package='bumperbot_controller').find('bumperbot_controller'),
        'config', 'bumperbot_controllers.yaml'
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    controllers_file = LaunchConfiguration('controllers_file')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot urdf file'
    )
    
    declare_controllers_file = DeclareLaunchArgument(
        name='controllers_file',
        default_value=controller_params_file,
        description='Path to controllers configuration file'
    )
    
    # Robot State Publisher - NOW WITH CONTROLLERS FILE
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', model, 
                ' controllers_file:=', controllers_file
            ]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Rest of the launch file remains the same...
    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(package='ros_gz_sim').find('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-name', 'bumperbot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )
    
    # Spawn Joint State Broadcaster with delay
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '20'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # Spawn Velocity Controller with additional delay
    spawn_velocity_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'simple_velocity_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '20'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_controllers_file,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_velocity_controller
    ])