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
        launch_arguments={"use_sim_time": "True"}.items()
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
        condition=IfCondition(use_slam)
    )

    # ---------------- SAFETY STOP ----------------
    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen"
    )

    # ---------------- RVIZ ----------------
    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            get_package_share_directory("bumperbot_localization"),
            "rviz",
            "odometry_motionModel.rviz"
        ])],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            get_package_share_directory("bumperbot_mapping"),
            "rviz",
            "slam.rviz"
        ])],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam
    ])
