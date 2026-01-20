#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = '/home/aun/Downloads/smr300l_gazebo_ros2control-main'
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params_working.yaml')
    
    # Read active map from Map Manager config
    active_map_config_file = os.path.join(pkg_dir, 'active_map_config.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'smr_map.yaml')
    
    try:
        if os.path.exists(active_map_config_file):
            with open(active_map_config_file, 'r') as f:
                config = yaml.safe_load(f) or {}
                map_file = config.get('active_map', default_map)
                print(f"[Navigation Composable Launch] Using active map: {map_file}")
        else:
            map_file = default_map
            print(f"[Navigation Composable Launch] No active map config, using default: {map_file}")
    except Exception as e:
        print(f"[Navigation Composable Launch] Error reading active map config: {e}, using default")
        map_file = default_map
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Create parameter rewrites with map file for map server
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={'yaml_filename': map_file},
        convert_types=True
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        
        # Container for Nav2 nodes
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        
        # Load Nav2 components into container
        LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='map_server',
                    parameters=[configured_params],
                    remappings=remappings
                ),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=remappings
                ),
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=remappings
                ),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[configured_params],
                    remappings=remappings
                ),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=remappings
                ),
            ],
        ),
        
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 10.0
            }]
        )
    ])
