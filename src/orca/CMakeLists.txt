cmake_minimum_required(VERSION 3.8)
project(orca)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # <-- ADD THIS

# Define the single executable target with all source files
add_executable(orca_node
  src/orca_main.cpp
  src/rvo2/Agent.cpp
  src/rvo2/KdTree.cpp
  src/rvo2/Obstacle.cpp
  src/rvo2/RVOSimulator.cpp
)

# Provide search paths for header resolution
target_include_directories(orca_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orca/rvo2>
)

# Use ROS 2 macro to automatically handle includes and links for all dependencies
ament_target_dependencies(orca_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs # <-- ADD THIS
)

# Install executable
install(TARGETS orca_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()