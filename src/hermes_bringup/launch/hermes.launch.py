from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    pkg_seq = ['spawn_bots', 'nav_start', 'nav_start', 'orca', 'multirobot_map_merge', 'frontier_allocator', 'frontier_exploration_ros2']
    launch_file_seq = ['spawn_bots.launch.py', 'slam_toolbox.launch.py', 'nav2.launch.py', 'orca.launch.py', 'map_merge.launch.py', 'frontier_allocator.launch.py', 
    'frontier_explorer.launch.py']
    robots = ['robot1', 'robot2']

    rviz_config_path = os.path.join(
        get_package_share_directory('hermes_bringup'),
        'rviz',
        'my_config.rviz'
    )

    for i in range(4):
        launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_seq[i]),
                'launch',
                launch_file_seq[i],
            )
        )
    )
        ld.add_action(
        TimerAction(
            period=float(i*10),
            actions=[launch_node]
        )
    )

    for i in range(len(robots)):
        robot_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"{robots[i]}_to_global_map",
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', f"{robots[i]}/map"
            ],
            output='screen'
        )
        ld.add_action(
            TimerAction(
                period=float((i+4) * 10),
                actions=[robot_tf]
            )
        )

    for i in range(3):
        launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_seq[i+4]),
                'launch',
                launch_file_seq[i+4],
            )
        )
    )
        ld.add_action(
        TimerAction(
            period=float((i+6) * 10),
            actions=[launch_node]
        )
    )
        

    # Define the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    ld.add_action(
        TimerAction(
            period=float(80),
            actions=[rviz_node]
        )
    )   
            
    return ld