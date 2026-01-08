import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

  # ONLY lifecycle-managed node
  lifecycle_nodes = [
      "slam_toolbox",
      "map_saver_server"
  ]

  use_sim_time_arg = DeclareLaunchArgument(
      "use_sim_time",
      default_value="true"
  )

  slam_config_arg = DeclareLaunchArgument(
    "slam_config",
    default_value=os.path.join(
      get_package_share_directory("bumperbot_mapping"),
      "config",
      "slam_toolbox.yaml"
    )
  )

  use_sim_time = LaunchConfiguration("use_sim_time")
  slam_config = LaunchConfiguration("slam_config")

  slam_toolbox = Node(
    package="slam_toolbox",
    executable="sync_slam_toolbox_node",
    name="slam_toolbox",
    output="screen",
    parameters=[
      slam_config,
      {"use_sim_time": use_sim_time}
    ]
  )

  map_saver_server = Node(
    package="nav2_map_server",
    executable="map_saver_server",
    name="map_saver_server",
    output="screen",
    parameters=[
      {"use_sim_time": use_sim_time},
      {"save_map_timeout": 5.0},
      {"free_thresh_default": 0.196},
      {"occupied_thresh_default": 0.65}
    ]
  )

  lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager",
    output="screen",
    parameters=[
      {"autostart": True},
      {"node_names": lifecycle_nodes},
      {"use_sim_time": use_sim_time}
    ]
  )

  return LaunchDescription([
    use_sim_time_arg,
    slam_config_arg,
    slam_toolbox,
    map_saver_server,
    lifecycle_manager
  ])
