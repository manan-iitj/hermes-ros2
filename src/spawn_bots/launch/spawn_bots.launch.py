import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def make_urdf(ns: str) -> str:
    """
    Generate a minimal URDF for robot_state_publisher.

    WHY THIS EXISTS:
      - Gazebo uses the SDF for physics and sensors (correct).
      - robot_state_publisher needs a URDF to publish static TF transforms
        between links (base_footprint→base_link→base_scan etc.).
      - turtlebot3_description is not installed on this system, so we
        generate the URDF programmatically with frame names that exactly
        match what the SDF's Gazebo plugins publish.

    FRAME NAMES must match the SDF exactly:
      - odometry_frame:     {ns}/odom
      - robot_base_frame:   {ns}/base_footprint
      - frame_name (lidar): {ns}/base_scan
    """
    
    return f"""<?xml version="1.0"?>
<robot name="{ns}">

  <!-- base_footprint is the root frame.
       Gazebo diff-drive plugin publishes {ns}/odom -> {ns}/base_footprint.
       RSP publishes everything below. -->
  <link name="{ns}/base_footprint"/>

  <joint name="{ns}/base_footprint_to_base_link" type="fixed">
    <parent link="{ns}/base_footprint"/>
    <child  link="{ns}/base_link"/>
    <origin xyz="0 0 0.010" rpy="0 0 0"/>
  </joint>
  <link name="{ns}/base_link"/>

  <!-- Laser scanner — frame_name in SDF: {ns}/base_scan -->
  <joint name="{ns}/base_link_to_base_scan" type="fixed">
    <parent link="{ns}/base_link"/>
    <child  link="{ns}/base_scan"/>
    <!-- TurtleBot3 Burger LiDAR offset (meters) -->
    <origin xyz="-0.032 0 0.172" rpy="0 0 0"/>
  </joint>
  <link name="{ns}/base_scan"/>

  <!-- IMU -->
  <joint name="{ns}/base_link_to_imu_link" type="fixed">
    <parent link="{ns}/base_link"/>
    <child  link="{ns}/imu_link"/>
    <origin xyz="-0.032 0 0.068" rpy="0 0 0"/>
  </joint>
  <link name="{ns}/imu_link"/>

  <!-- Caster wheel -->
  <joint name="{ns}/base_link_to_caster" type="fixed">
    <parent link="{ns}/base_link"/>
    <child  link="{ns}/caster_back_link"/>
    <origin xyz="-0.081 0 -0.004" rpy="-1.5707963268 0 0"/>
  </joint>
  <link name="{ns}/caster_back_link"/>

  <!-- Left wheel (continuous - joint states from Gazebo fill this) -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="{ns}/base_link"/>
    <child  link="{ns}/wheel_left_link"/>
    <origin xyz="0 0.080 0.023" rpy="-1.5707963268 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="{ns}/wheel_left_link"/>

  <!-- Right wheel -->
  <joint name="wheel_right_joint" type="continuous">
    <parent link="{ns}/base_link"/>
    <child  link="{ns}/wheel_right_link"/>
    <origin xyz="0 -0.080 0.023" rpy="-1.5707963268 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="{ns}/wheel_right_link"/>

</robot>
"""


def generate_launch_description():

    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    my_pkg_share   = get_package_share_directory('spawn_bots')
    custom_world   = os.path.join(my_pkg_share, 'world', 'my_world.world')

    # Robot roster
    robots = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.1'},
        {'name': 'robot2', 'x': '3.5', 'y': '2.5', 'z': '0.1'},
    ]

    # Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': custom_world}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Per-robot nodes
    for robot in robots:
        ns    = robot['name']
        model = os.path.join(my_pkg_share, 'models', f'{ns}.sdf')

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{ns}_rsp',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': make_urdf(ns),
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        )

        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity',          ns,
                '-file',            model,
                '-robot_namespace', ns,
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', robot['z'],
            ],
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen'
        )

        ld.add_action(rsp)
        ld.add_action(spawn)

    return ld