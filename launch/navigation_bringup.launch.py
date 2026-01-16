#!/usr/bin/env python3
"""
Wrapper for nav2_bringup that reads active map from Map Manager config
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = '/home/aun/Downloads/smr300l_gazebo_ros2control-main'
    
    # Read active map from Map Manager config
    active_map_config_file = os.path.join(pkg_dir, 'active_map_config.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'smr_map.yaml')
    
    try:
        if os.path.exists(active_map_config_file):
            with open(active_map_config_file, 'r') as f:
                config = yaml.safe_load(f) or {}
                map_file = config.get('active_map', default_map)
                print(f"[Navigation Bringup Wrapper] Using active map: {map_file}")
        else:
            map_file = default_map
            print(f"[Navigation Bringup Wrapper] No active map config, using default: {map_file}")
    except Exception as e:
        print(f"[Navigation Bringup Wrapper] Error reading active map config: {e}, using default")
        map_file = default_map
    
    # Get nav2_bringup package path
    nav2_bringup_dir = '/opt/ros/humble/share/nav2_bringup/launch'
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('params_file', 
                            default_value=os.path.join(pkg_dir, 'config', 'nav2_params_working.yaml'),
                            description='Full path to the ROS2 parameters file to use'),
        
        # Include nav2_bringup with the dynamically determined map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'use_rviz': 'False'
            }.items()
        )
    ])
