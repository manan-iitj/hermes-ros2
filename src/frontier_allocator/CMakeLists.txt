cmake_minimum_required(VERSION 3.8)
project(frontier_allocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# -----------------------------------------------------------------------
# C++ executables -- built unconditionally (NOT gated behind BUILD_TESTING,
# which only controls whether tests run; nodes must always be built).
# -----------------------------------------------------------------------

add_executable(frontier_alg src/frontier_alg.cpp)
ament_target_dependencies(frontier_alg
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
)

add_executable(frontier_pub src/frontier_pub.cpp)
ament_target_dependencies(frontier_pub
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  frontier_alg
  frontier_pub
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------
# Testing / linting only -- does not affect whether the nodes above build.
# -----------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()