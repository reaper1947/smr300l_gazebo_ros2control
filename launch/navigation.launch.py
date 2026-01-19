#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = '/home/aun/Downloads/smr300l_gazebo_ros2control-main'
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Read active map from Map Manager config
    active_map_config_file = os.path.join(pkg_dir, 'active_map_config.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'smr_map.yaml')
    
    try:
        if os.path.exists(active_map_config_file):
            with open(active_map_config_file, 'r') as f:
                config = yaml.safe_load(f) or {}
                map_file = config.get('active_map', default_map)
                print(f"[Navigation Launch] Using active map: {map_file}")
        else:
            map_file = default_map
            print(f"[Navigation Launch] No active map config, using default: {map_file}")
    except Exception as e:
        print(f"[Navigation Launch] Error reading active map config: {e}, using default")
        map_file = default_map
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Start bt_navigator LAST after others are active
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[nav2_params_file, {'yaml_filename': map_file}], remappings=remappings),
        
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[nav2_params_file], remappings=remappings),
        
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[nav2_params_file], remappings=remappings),
        
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[nav2_params_file], remappings=remappings),
        
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[nav2_params_file], remappings=remappings),
        
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 
             'node_names': lifecycle_nodes, 'bond_timeout': 10.0, 'attempt_respawn_reconnection': True,
             'bond_respawn_max_duration': 10.0}])
    ])
