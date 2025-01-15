cmake_minimum_required(VERSION 3.8)
project(kmriiwa_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(launch)
endif()

ament_package()
