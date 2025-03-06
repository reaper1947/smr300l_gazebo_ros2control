import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    package_name = 'my_bot'  # <--- CHANGE ME

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    controller_manager = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "my_controllers.yaml")],
            output="screen"
        )]
    )

    diff_drive_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        )]
    )

    joint_broad_spawner = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        )]
    )

    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner
    ])
