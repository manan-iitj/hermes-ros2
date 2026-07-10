"""
Launch file for the 3-part Hungarian frontier allocator.

Starts, in order:
  1. part1_subscriber  - listens to explore_lite frontiers + robot odom/TF
  2. part2_algorithm   - runs the Hungarian algorithm
  3. part3_publisher   - sends the resulting goal to each robot

Run with:
    ros2 --no-daemon launch frontier_allocator frontier_allocator_launch.py

(Replace 'frontier_allocator' below with your actual package name if different.)

NOTE: this is a C++ (ament_cmake) package, not ament_python. There is no
setup.py/entry_points here -- the Python scripts are installed directly via
CMakeLists.txt's install(PROGRAMS ...) command, so the 'executable' name below
must match the script's exact filename, including the .py extension.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    part1 = Node(
        package='frontier_allocator',
        executable='frontier_alg',
        name='frontier_alg',
        output='screen',
    )

    part2 = Node(
        package='frontier_allocator',
        executable='frontier_pub',
        name='frontier_pub',
        output='screen',
    )

    return LaunchDescription([part1, part2])