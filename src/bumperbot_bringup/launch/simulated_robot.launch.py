import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------- ARGUMENT ----------------
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_teleop_arg = DeclareLaunchArgument(
        "use_teleop",
        default_value="false"
    )    

    use_teleop = LaunchConfiguration("use_teleop")

    use_slam = LaunchConfiguration("use_slam")

    # ---------------- GAZEBO ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_description"),
                "launch",
                "gazebo.launch.py"
            ])
        )
    )

    # ---------------- CONTROLLERS ----------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controllers.launch.py"
            ])
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    # ---------------- JOYSTICK ----------------
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            ])
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
        condition=IfCondition(use_teleop)
    )

    # ---------------- LOCALIZATION / SLAM ----------------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "global_localization.launch.py"
            ])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_mapping"),
                "launch",
                "slam.launch.py"
            ])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("bumperbot_navigation"),
                "launch",
                "navigation.launch.py"
            ])
        ),
        launch_arguments={
                    "use_sim_time": "true",
                    "cmd_vel_topic": "/bumperbot_controller/cmd_vel" 
                }.items()
    )

    # ---------------- SAFETY STOP ----------------
    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen",
        condition=IfCondition(use_teleop)
    )

    # ---------------- RVIZ ----------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            get_package_share_directory("nav2_bringup"),
            "rviz",
            "nav2_default_view.rviz"
        ])],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    return LaunchDescription([
        use_slam_arg,
        use_teleop_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        navigation,
        rviz
    ])
