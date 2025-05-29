cmake_minimum_required(VERSION 3.8)
project(kmriiwa_arm_control)

# Find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install directories
install(
  DIRECTORY
    scripts
  DESTINATION share/${PROJECT_NAME}
)


# Instead of ament_python_install_package
install(PROGRAMS
  scripts/kmriiwa_arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
