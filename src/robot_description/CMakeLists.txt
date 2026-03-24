cmake_minimum_required(VERSION 3.5)
project(robot_description VERSION 0.0.1 LANGUAGES CXX)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_msgs
  generate_parameter_library
  controller_interface
  realtime_tools
  std_msgs
)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)        # simulator interface

find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  robot_description
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(robot_description PUBLIC cxx_std_17)
target_include_directories(robot_description PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_description>
)
ament_target_dependencies(
  robot_description PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface robot_description.xml)

# Add library of the controller and export it
generate_parameter_library(passthrough_controller_parameters
  controllers/src/passthrough_controller_parameters.yaml
)

add_library(passthrough_controller SHARED
  controllers/src/passthrough_controller.cpp
)
target_include_directories(passthrough_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/passthrough_controller>
)
target_link_libraries(passthrough_controller PUBLIC passthrough_controller_parameters)
ament_target_dependencies(passthrough_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface passthrough_controller.xml)


install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_description
)

install(
  DIRECTORY controllers/include/
  DESTINATION include/passthrough_controller
)

install(TARGETS
    passthrough_controller
    passthrough_controller_parameters
  EXPORT export_passthrough_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


# Install URDF, meshes, launch files, and config
install(DIRECTORY launch config rviz models worlds urdf gazebo hardware 
  DESTINATION share/${PROJECT_NAME}
)

# Declare this package as an ament package
ament_package()
