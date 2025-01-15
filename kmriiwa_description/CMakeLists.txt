cmake_minimum_required(VERSION 3.8)
project(kmriiwa_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF, meshes, and other description files
install(
  DIRECTORY 
    urdf
    meshes    
  DESTINATION 
    share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Necessary for the package to be found by ament
ament_package()
