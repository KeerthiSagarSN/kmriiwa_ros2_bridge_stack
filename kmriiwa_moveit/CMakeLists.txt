cmake_minimum_required(VERSION 3.8)
project(kmriiwa_moveit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files (excluding setup_assistant.launch)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Necessary for the package to be found by ament
ament_package()
