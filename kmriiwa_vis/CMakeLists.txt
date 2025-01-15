cmake_minimum_required(VERSION 3.8)
project(kmriiwa_vis)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Launch testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(launch)
endif()

ament_package()
