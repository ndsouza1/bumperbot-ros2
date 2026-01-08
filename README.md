# Bumperbot ROS 2 Project

A differential drive robot simulation project built with ROS 2, Gazebo, and RViz2. This workspace contains packages for robot description, control, localization, mapping, and example implementations of common ROS 2 concepts.

## Overview

This project implements a complete ROS 2 simulation environment for a differential drive robot called Bumperbot. The robot is equipped with an IMU sensor and a laser scanner. The workspace includes functionality for robot control, sensor integration, SLAM, localization, and navigation.

The project demonstrates practical implementations of ROS 2 concepts including publishers, subscribers, services, actions, parameters, lifecycle nodes, transforms, and quality of service settings.

## Project Structure

The workspace is organized into the following packages:

### Core Robot Packages

**bumperbot_description**
Contains URDF/Xacro files defining the robot model, meshes, Gazebo world files, and RViz configurations. Includes launch files for visualizing the robot in RViz2 and spawning it in Gazebo.

**bumperbot_bringup**
Provides high-level launch files to bring up the complete simulated or real robot system. Includes configurations for launching Gazebo, controllers, localization, SLAM, and visualization tools together.

**bumperbot_controller**
Implements robot control functionality including differential drive controllers, joystick teleoperation, and twist multiplexing. Contains both C++ and Python implementations of control nodes.

**bumperbot_firmware**
Hardware interface layer for communicating with the physical robot hardware. Includes firmware code and hardware abstraction for real robot deployment.

### Localization and Mapping

**bumperbot_localization**
Implements robot localization using sensor fusion. Includes configurations for both local and global localization using the robot_localization package with IMU and odometry data.

**bumperbot_mapping**
SLAM implementation using slam_toolbox for creating maps of the environment. Provides launch files and configurations for online SLAM.

### Example Packages

**bumperbot_cpp_examples**
C++ implementations demonstrating ROS 2 concepts:
- Publishers and subscribers
- Service clients and servers
- Action clients and servers
- Parameters and dynamic reconfiguration
- Lifecycle nodes
- TF transformations and kinematics
- Quality of Service settings

**bumperbot_py_examples**
Python implementations of similar ROS 2 concepts for comparison with the C++ versions.

### Utilities

**bumperbot_msgs**
Custom message and service definitions used across the project.

**bumperbot_utils**
Utility nodes including a safety stop feature for collision avoidance.

**twist_relay**
Node for relaying and processing twist messages for robot velocity control.

## Tools and Technologies

**ROS 2**
Primary framework for robot software development.

**Gazebo**
Physics-based simulation environment using ros_gz_sim for integration with ROS 2.

**RViz2**
3D visualization tool for robot state, sensor data, and navigation information.

**ros2_control**
Framework for robot control with support for different controller types.

**robot_localization**
Extended Kalman Filter implementation for sensor fusion and state estimation.

**slam_toolbox**
SLAM implementation for mapping and localization.

**Navigation2**
Map server and lifecycle management for navigation tasks.

## Building the Project

Navigate to the workspace root and build all packages:

```bash
cd /home/nel/Documents/bumperbo_Ws
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

## Running the Simulation

Launch the complete simulated robot with localization:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

Launch the simulated robot with SLAM:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

Launch only Gazebo with the robot:

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

Launch only RViz for visualization:

```bash
ros2 launch bumperbot_description display.launch.py
```

## Running Example Nodes

The example packages demonstrate various ROS 2 features. For instance, to run the simple publisher and subscriber in C++:

```bash
ros2 run bumperbot_cpp_examples simple_publisher
ros2 run bumperbot_cpp_examples simple_subscriber
```

Or the Python versions:

```bash
ros2 run bumperbot_py_examples simple_publisher
ros2 run bumperbot_py_examples simple_subscriber
```

## Robot Control

Control the robot using a joystick:

```bash
ros2 launch bumperbot_controller joystick_teleop.launch.py
```

The robot can be controlled through velocity commands published to the appropriate twist topics, with twist_mux handling multiple command sources.

## Mapping and Localization

The project supports both SLAM for creating maps and localization for navigating with existing maps. The simulated_robot launch file handles switching between these modes using the use_slam argument.

A snapshot of the robot performing mapping and navigation is available in the snapshot_map_navigation folder.

![Mapping and Navigation](snapshot_map_navigation/map_gz_rviz.png)

## What Was Learned

This project provided hands-on experience with:

- Designing and describing robots using URDF and Xacro
- Integrating robots with Gazebo simulation
- Implementing differential drive control systems
- Working with ROS 2 publishers, subscribers, services, and actions
- Understanding and configuring Quality of Service policies
- Using lifecycle nodes for managed node states
- Performing coordinate transformations with TF2
- Implementing sensor fusion for localization
- Creating maps using SLAM
- Structuring multi-package ROS 2 workspaces
- Writing launch files for complex system bringup

## Author

Nel
