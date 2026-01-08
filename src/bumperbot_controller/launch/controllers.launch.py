from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")

    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {
                "wheel_radius": wheel_radius + wheel_radius_error,
                "wheel_separation": wheel_separation + wheel_separation_error,
                "use_sim_time": use_sim_time
            }
        ],
        condition=IfCondition(use_python)
    )

    noisy_controller_cpp = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[
            {
                "wheel_radius": wheel_radius + wheel_radius_error,
                "wheel_separation": wheel_separation + wheel_separation_error,
                "use_sim_time": use_sim_time
            }
        ],
        condition=UnlessCondition(use_python)
    )

    return [
        noisy_controller_py,
        noisy_controller_cpp
    ]


def generate_launch_description():

    # -----------------------------
    # Launch Arguments
    # -----------------------------
    controller_config_arg = DeclareLaunchArgument(
        "controller_config_file",
        default_value="/home/nel/bumperbo_ws/src/bumperbot_controller/config/bumperbot_controllers.yaml"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller",
        default_value="False",
        description="Enable noisy controller (localization mode only)"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    # -----------------------------
    # Launch Configurations
    # -----------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_noisy_controller = LaunchConfiguration("use_noisy_controller")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # -----------------------------
    # Controllers
    # -----------------------------
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "20"
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen"
            )
        ]
    )

    wheel_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "bumperbot_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "20",
                    "--ros-args", "--params-file",
                    LaunchConfiguration("controller_config_file")
                ],
                condition=UnlessCondition(use_simple_controller),
                output="screen"
            )
        ]
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "simple_velocity_controller",
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "20"
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen"
                    )
                ]
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[
                    {
                        "wheel_radius": wheel_radius,
                        "wheel_separation": wheel_separation,
                        "use_sim_time": use_sim_time
                    }
                ],
                condition=IfCondition(use_python)
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[
                    {
                        "wheel_radius": wheel_radius,
                        "wheel_separation": wheel_separation,
                        "use_sim_time": use_sim_time
                    }
                ],
                condition=UnlessCondition(use_python)
            )
        ]
    )

    # -----------------------------
    # 🔥 Noisy Controller (CONDITIONAL)
    # -----------------------------
    noisy_controller_launch = GroupAction(
        condition=IfCondition(use_noisy_controller),
        actions=[
            OpaqueFunction(function=noisy_controller)
        ]
    )

    # -----------------------------
    # Launch Description
    # -----------------------------
    return LaunchDescription([
        controller_config_arg,
        use_sim_time_arg,
        use_python_arg,
        use_simple_controller_arg,
        use_noisy_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        simple_controller,
        wheel_controller_spawner,
        joint_state_broadcaster_spawner,
        noisy_controller_launch
    ])
