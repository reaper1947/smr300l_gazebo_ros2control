#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of this launch file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    return LaunchDescription([
        # Include the navigation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation.launch.py')
            )
        ),
        
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
