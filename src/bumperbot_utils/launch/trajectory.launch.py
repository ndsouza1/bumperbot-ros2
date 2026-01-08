from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    odom_topic = LaunchConfiguration('odom_topic')
    trajectory_topic = LaunchConfiguration('trajectory_topic')
    max_trajectory_points = LaunchConfiguration('max_trajectory_points')
    use_sim_time = LaunchConfiguration('use_sim_time')


    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/bumperbot_controller/odom',
        description='Odometry topic to subscribe to'
    )
    
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/bumperbot_controller/trajectory',
        description='Topic to publish trajectory path'
    )
    
    max_trajectory_points_arg = DeclareLaunchArgument(
        'max_trajectory_points',
        default_value='1000',
        description='Maximum number of points to keep in trajectory'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
          

    # C++ trajectory drawer node
    simple_trajectory_cpp = Node(
        package='bumperbot_utils',
        executable='simple_trajectory',
        name='simple_trajectory',
        parameters=[
            {
                'odom_topic': odom_topic,
                'trajectory_topic': trajectory_topic,
                'max_trajectory_points': max_trajectory_points,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        odom_topic_arg,
        trajectory_topic_arg,
        max_trajectory_points_arg,
        use_sim_time_arg,
        simple_trajectory_cpp
    ])