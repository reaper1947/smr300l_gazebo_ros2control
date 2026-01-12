import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

<<<<<<< HEAD
    # Launch!
=======
    # Remove controller spawner from sim launch
    # node_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_controller', 'robot_state_controller'],
    #     output='screen',
    #     parameters=[controllers_yaml]
    # )

    # Launch!
    # Only robot_state_publisher (and optionally joint_state_publisher) should be launched in sim
>>>>>>> 51bf80f (web ui nav update)
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
<<<<<<< HEAD
        node_joint_state_publisher,
        node_robot_state_publisher

=======
        # node_joint_state_publisher,  # Optionally comment this out for sim
        node_robot_state_publisher
>>>>>>> 51bf80f (web ui nav update)
    ])
