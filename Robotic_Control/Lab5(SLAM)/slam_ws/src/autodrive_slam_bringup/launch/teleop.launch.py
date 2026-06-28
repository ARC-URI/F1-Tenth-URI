#!/usr/bin/env python3
"""
teleop.launch.py

Standalone launch for the Ackermann keyboard teleop node. Run this in its
own terminal (so it has keyboard focus) alongside slam_bringup.launch.py.

Usage:
  ros2 launch autodrive_slam_bringup teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='autodrive_slam_bringup',
        executable='ackermann_keyboard_teleop.py',
        name='ackermann_keyboard_teleop',
        output='screen',
    )
    return LaunchDescription([teleop_node])
