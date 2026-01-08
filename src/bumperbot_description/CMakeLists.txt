cmake_minimum_required(VERSION 3.8)
project(bumperbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  

# Install URDF, meshes, launch, etc.
install(
  DIRECTORY meshes urdf launch rviz models worlds photos
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(
  PROGRAMS
    scripts/imu_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_bridge
)

ament_package()