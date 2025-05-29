cmake_minimum_required(VERSION 3.5)
project(kmriiwa_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## Set dependencies

set(dependencies
  rclcpp
  rclpy
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros  
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (if any)
#install(
#  DESTINATION share/${PROJECT_NAME}
#  OPTIONAL
#)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Necessary for the package to be found by ament
ament_export_dependencies(${dependencies})

ament_package()
