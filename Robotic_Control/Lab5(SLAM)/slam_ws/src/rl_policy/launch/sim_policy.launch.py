"""Run the trained policy in the AutoDRIVE simulator.

Starts autodrive_adapter_node (AutoDRIVE topics <-> /scan, /odom, /drive) and
the policy node. The AutoDRIVE bridge and simulator must already be running.

Usage:
  ros2 launch rl_policy sim_policy.launch.py
  ros2 launch rl_policy sim_policy.launch.py max_speed:=2.0
  ros2 launch rl_policy sim_policy.launch.py weights_path:=/path/to/f1tenth_policy.npz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topics_yaml = os.path.join(
        get_package_share_directory('autodrive_slam_bringup'), 'config', 'topics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('weights_path', default_value='',
                              description='Exported f1tenth_policy.npz; empty uses the one in this package'),
        DeclareLaunchArgument('max_speed', default_value='1.0', description='Speed cap in m/s'),
        DeclareLaunchArgument('control_rate', default_value='0.0',
                              description='Control rate limit in Hz; 0 uses the trained rate'),
        DeclareLaunchArgument('sensor_timeout', default_value='1.5',
                              description='Seconds without data before stopping (the simulator is slow)'),
        Node(
            package='autodrive_slam_bringup',
            executable='autodrive_adapter_node.py',
            name='autodrive_adapter_node',
            output='screen',
            parameters=[topics_yaml],
        ),
        Node(
            package='rl_policy',
            executable='policy_node',
            output='screen',
            parameters=[{
                'weights_path': LaunchConfiguration('weights_path'),
                'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
                'control_rate': ParameterValue(LaunchConfiguration('control_rate'), value_type=float),
                'sensor_timeout': ParameterValue(LaunchConfiguration('sensor_timeout'), value_type=float),
            }],
        ),
    ])
