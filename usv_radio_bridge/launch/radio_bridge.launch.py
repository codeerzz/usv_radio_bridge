"""
RFD900x radio bridge — vehicle-side link to the ground-station GUI.

  ros2 launch usv_radio_bridge radio_bridge.launch.py

Included by usv_bringup.launch.py's REAL branch (radio_bridge:=true by
default there); never a required dependency for Nav2/localization/GNC, so
it's also fine to run standalone for bench testing the radio link alone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    radio_bridge_params = os.path.join(
        get_package_share_directory('usv_radio_bridge'), 'config', 'radio_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='usv_radio_bridge',
            executable='radio_bridge_node',
            name='radio_bridge_node',
            output='screen',
            parameters=[radio_bridge_params, {'use_sim_time': use_sim_time}],
        ),
    ])
