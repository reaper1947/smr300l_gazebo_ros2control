import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)

    # -------- Launch arguments --------
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'home/aun/Downloads/small_warehouse.world'),
        description='Full path to Gazebo world file (.world or .sdf)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # -------- Robot state publisher / ros2_control description --------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true'
        }.items()
    )

    # -------- Joystick + twist mux --------
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # -------- Gazebo --------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    # -------- Spawn robot into Gazebo --------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1']
    )

    # -------- Controllers (spawn AFTER spawn_entity starts) --------
    # This is much more reliable than fixed TimerAction delays.
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[joint_broad_spawner, diff_drive_spawner]
        )
    )

    # -------- Safety Controller + Zone Publisher (Keepout & Speed Filters) --------
    safety_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'safety_controller.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time,

        rsp,
        joystick,
        twist_mux,
        safety_controller,

        gazebo,
        spawn_entity,

        start_controllers_after_spawn,
    ])