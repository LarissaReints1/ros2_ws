cmake_minimum_required(VERSION 3.5)
project(droneurdf2 VERSION 0.0.1 LANGUAGES CXX)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)        # simulator interface

find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

# Install URDF, meshes, launch files, and config
install(DIRECTORY urdf meshes launch config rviz models sdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Declare this package as an ament package
ament_package()
