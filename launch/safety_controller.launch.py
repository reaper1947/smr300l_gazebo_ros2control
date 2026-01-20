#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next_ros2ws_core',
            executable='safety_controller',
            name='safety_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
    ])
