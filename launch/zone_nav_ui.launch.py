#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of this launch file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    return LaunchDescription([
        # NOTE: Nav2 navigation stack should already be running from your bringup command
        # This launch file ONLY starts the UI and zone management nodes
        
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
        
        # Safety controller node - monitors obstacles and localization
        Node(
            package='zone_nav',
            executable='safety_controller',
            name='safety_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # Scan merger node - combines /scan + /scan2 for AMCL
        Node(
            package='zone_nav',
            executable='scan_merger',
            name='scan_merger',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # Map manager node - centralized map management service
        Node(
            package='zone_nav',
            executable='map_manager',
            name='map_manager',
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
