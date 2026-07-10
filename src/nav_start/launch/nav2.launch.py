import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_start')

    robots = [{'name': 'robot1'}, {'name': 'robot2'}]
    robot_actions = []

    # Standard lifecycle nodes managed by the Nav2 lifecycle manager
    nav2_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    for robot in robots:
        namespace = robot['name']
        bot_params = os.path.join(pkg_dir, 'config', f"{namespace}_nav2.yaml")

        # Explicit configuration for each of the core Nav2 servers
        nodes_to_launch = [
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_raw')
                ]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[bot_params, {'use_sim_time': True}],
            ),
            # The Lifecycle Manager orchestrates transitions (Unconfigured -> Active)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': nav2_lifecycle_nodes
                }]
            ),
        ]

        # Wrap everything for this robot in a GroupAction to apply the namespace cleanly
        bot_nav = GroupAction([
            PushRosNamespace(namespace),
            *nodes_to_launch
        ])
        
        robot_actions.append(bot_nav)

    return LaunchDescription(robot_actions)