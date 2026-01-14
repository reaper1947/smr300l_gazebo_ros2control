#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = '/home/aun/Downloads/smr300l_gazebo_ros2control-main'
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params_working.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'smr_map.yaml')
    
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
