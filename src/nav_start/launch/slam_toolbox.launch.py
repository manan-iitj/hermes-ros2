import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('nav_start')

    slam_params = os.path.join(
        pkg_share,
        'config',
        'multi_bot_slam.yaml'
    )

    robots = [
        "robot1",
        "robot2"
    ]

    ld = LaunchDescription()

    for robot in robots:

        slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot,
        output="screen",

        parameters=[
            slam_params,
            {
                "use_sim_time": True,
                # Force the internal SLAM tracking frames to match your namespaced TF tree
                "map_frame": f"{robot}/map",
                "odom_frame": f"{robot}/odom",
                "base_frame": f"{robot}/base_footprint",
                "scan_topic": "scan"  # Evaluates to /{robot}/scan automatically inside the namespace
            }
        ],

        remappings=[
            # Explicit namespaced map routing
            ("/map", f"/{robot}/map"),
            ("/map_metadata", f"/{robot}/map_metadata"),
        ]
    )

        ld.add_action(slam)

    return ld