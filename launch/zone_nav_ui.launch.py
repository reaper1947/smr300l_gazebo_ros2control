#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Zone manager node
        Node(
            package='zone_nav',
            executable='zone_manager',
            name='zone_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # Web UI node
        Node(
            package='zone_nav',
            executable='zone_web_ui',
            name='zone_web_ui',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
    ])
