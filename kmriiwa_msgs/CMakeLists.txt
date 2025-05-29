cmake_minimum_required(VERSION 3.8)
project(kmriiwa_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Prepare message files with absolute path
set(msg_files)
file(GLOB _msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
foreach(_msg_file ${_msg_files})
  list(APPEND msg_files "${CMAKE_CURRENT_SOURCE_DIR}:${_msg_file}")
endforeach()

# Generate ROS interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES 
    std_msgs 
    geometry_msgs 
    sensor_msgs
)

# Export dependencies
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs 
  sensor_msgs
)

# Installation
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Necessary for the package to be found by ament
ament_package()
