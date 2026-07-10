from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    orca_launch = Node(
        package='orca',
        executable='orca_node',
        name='orca_node',
        output='screen',
    )

    return LaunchDescription([orca_launch])