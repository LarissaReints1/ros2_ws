cmake_minimum_required(VERSION 3.5)
project(robot_description VERSION 0.0.1 LANGUAGES CXX)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

# Install URDF, meshes, launch files, config, rviz
install(DIRECTORY launch config rviz models  
  DESTINATION share/${PROJECT_NAME}
)

# Declare this package as an ament package
ament_package()