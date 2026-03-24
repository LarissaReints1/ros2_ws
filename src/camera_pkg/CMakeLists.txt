cmake_minimum_required(VERSION 3.8)
project(camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cam2image_node src/cam2image_node.cpp)
ament_target_dependencies(cam2image_node rclcpp sensor_msgs cv_bridge OpenCV camera_info_manager)

target_link_libraries(cam2image_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  cam2image_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()