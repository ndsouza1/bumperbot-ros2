import os
from os import pathsep
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    os.environ['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = 'ERROR'
    
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
    
    model_path = str(Path(pkg_share).parent.resolve())
    model_path += pathsep + os.path.join(pkg_share, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path
    )

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

    declare_world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty.world",   # <-- use .world directly
        description="World file name including .world extension"
    )

    world_path = PathJoinSubstitution([
        pkg_share,
        "worlds",
        LaunchConfiguration("world_name")
    ])
        
    declare_controllers_file = DeclareLaunchArgument(
        name='controllers_file',
        default_value=controller_params_file,
        description='Path to controllers configuration file'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', model, 
                ' is_sim:=true',           
                ' is_ignition:=false',     
                ' controllers_file:=', controllers_file
            ]),
            'use_sim_time': use_sim_time
        }]
    )
        
    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(package='ros_gz_sim').find('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v 4 '), world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gz_ros2_clock_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        arguments=['clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], 
        parameters=[{'use_sim_time': use_sim_time}], 
        output='screen'
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

    gz_ros2_imu_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    gz_ros2_laser_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],  
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_world_name_arg,
        declare_controllers_file,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo_launch,
        gz_ros2_clock_bridge,
        spawn_robot,
        gz_ros2_imu_bridge,
        gz_ros2_laser_bridge
    ])
