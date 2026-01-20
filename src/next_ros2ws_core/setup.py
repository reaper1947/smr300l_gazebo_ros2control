from setuptools import setup

package_name = 'next_ros2ws_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aun',
    maintainer_email='aun@todo.todo',
    description='Core ROS2 nodes for next_ros2ws (no web/heavy deps).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keepout_zone_publisher = next_ros2ws_core.keepout_zone_publisher:main',
            'map_manager           = next_ros2ws_core.map_manager:main',
            'mode_switcher         = next_ros2ws_core.mode_switcher:main',
            'safety_controller     = next_ros2ws_core.safety_controller:main',
            'scan_merger           = next_ros2ws_core.scan_merger:main',
            'stack_manager         = next_ros2ws_core.stack_manager:main',
            'zone_manager          = next_ros2ws_core.zone_manager:main',
        ],
    },
)

