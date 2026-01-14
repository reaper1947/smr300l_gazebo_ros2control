#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zone_nav',
            executable='mode_switcher',
            name='mode_switcher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
