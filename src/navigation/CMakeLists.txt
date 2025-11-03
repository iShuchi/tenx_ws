cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Path smoother library
add_library(path_smoother SHARED
  src/path_smoother/simple_smoother.cpp
  src/path_smoother/savgol_smoother.cpp
)

ament_target_dependencies(path_smoother
  rclcpp
  geometry_msgs
  nav_msgs
)

# Trajectory generator library
add_library(trajectory_generator SHARED
  src/trajectory_generator/trajectory_generator.cpp
)

ament_target_dependencies(trajectory_generator
  rclcpp
  geometry_msgs
  nav_msgs
)

# Controller library
add_library(controller SHARED
  src/controller/pure_pursuit_controller.cpp
)

ament_target_dependencies(controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Main node executable
add_executable(path_tracking_node
  src/path_tracking_node.cpp
)

ament_target_dependencies(path_tracking_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(path_tracking_node
  path_smoother
  trajectory_generator
  controller
)

# Install targets
install(TARGETS
  path_smoother
  trajectory_generator
  controller
  DESTINATION lib
)

install(TARGETS
  path_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

