#!/usr/bin/env python3
"""
slam_bringup.launch.py

Brings up SLAM for the Ackermann-steered AutoDRIVE F1TENTH vehicle:
  1. autodrive_adapter_node  - normalizes AutoDRIVE topics to /scan, /odom,
                                TF, and converts /drive (AckermannDriveStamped)
                                into AutoDRIVE throttle/steering commands.
  2. static_transform_publisher - base_link -> laser (LiDAR mount offset).
  3. slam_toolbox (async_slam_toolbox_node) - builds the occupancy map live.
  4. rviz2 (optional) - preloaded visualization.

PREREQUISITE: AutoDRIVE Simulator + its ROS 2 bridge (autodrive_f1tenth)
must already be running before (or alongside) this launch file, since this
package only adapts/consumes its topics -- it does not launch the simulator
itself. See the README in this package for the AutoDRIVE-side launch
command.

Usage:
  ros2 launch autodrive_slam_bringup slam_bringup.launch.py
  ros2 launch autodrive_slam_bringup slam_bringup.launch.py use_rviz:=false
  ros2 launch autodrive_slam_bringup slam_bringup.launch.py use_teleop:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('autodrive_slam_bringup')

    topics_yaml = os.path.join(pkg_share, 'config', 'topics.yaml')
    slam_yaml = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with a preloaded SLAM view.'
    )
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='false',
        description='Launch the WASD AckermannDriveStamped keyboard teleop '
                     'node in this same launch (run in its own terminal '
                     'with focus if you want responsive keypresses).'
    )
    lidar_x_offset_arg = DeclareLaunchArgument(
        'lidar_x_offset', default_value='0.27',
        description='LiDAR mount X offset from base_link, meters (F1TENTH default: near front).'
    )
    lidar_z_offset_arg = DeclareLaunchArgument(
        'lidar_z_offset', default_value='0.10',
        description='LiDAR mount Z offset (height) from base_link, meters.'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    lidar_x_offset = LaunchConfiguration('lidar_x_offset')
    lidar_z_offset = LaunchConfiguration('lidar_z_offset')

    adapter_node = Node(
        package='autodrive_slam_bringup',
        executable='autodrive_adapter_node.py',
        name='autodrive_adapter_node',
        output='screen',
        parameters=[topics_yaml],
    )

    # Static transform: base_link -> laser. Adjust offsets to match your
    # vehicle's actual LiDAR mount position (see Car-Parameters.md in the
    # AutoDRIVE-F1TENTH repo for the stock mount geometry).
    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            lidar_x_offset, '0', lidar_z_offset,   # x y z
            '0', '0', '0',                          # roll pitch yaw
            'base_link', 'laser'
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
        # Forces the Mesa software rasterizer. WSL's virtualized GPU driver
        # has a known shader-link bug on rviz's indexed_8bit_image shader
        # (used by the Map display), which the software path avoids.
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    )

    teleop_node = Node(
        package='autodrive_slam_bringup',
        executable='ackermann_keyboard_teleop.py',
        name='ackermann_keyboard_teleop',
        output='screen',
        prefix='xterm -e',   # needs its own terminal for raw keyboard input
        condition=IfCondition(use_teleop),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_teleop_arg,
        lidar_x_offset_arg,
        lidar_z_offset_arg,
        adapter_node,
        laser_static_tf,
        slam_toolbox_node,
        rviz_node,
        teleop_node,
    ])
