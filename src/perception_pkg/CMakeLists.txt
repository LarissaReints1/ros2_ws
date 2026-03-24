cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

# TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/perception_pkg
  
)

add_executable(perception_node src/perception_node.cpp src/helper_extract_target.cpp)

ament_target_dependencies(
  perception_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)



install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
