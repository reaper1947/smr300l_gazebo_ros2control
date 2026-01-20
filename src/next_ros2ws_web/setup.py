from setuptools import setup
import os
from glob import glob

package_name = 'next_ros2ws_web'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install web assets
        (os.path.join('share', package_name, 'web', 'templates'), glob('next_ros2ws_web/web/templates/*')),
        (os.path.join('share', package_name, 'web', 'static'), glob('next_ros2ws_web/web/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aun',
    maintainer_email='aun@todo.todo',
    description='Web UI for next_ros2ws (Flask).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_web_ui = next_ros2ws_web.zone_web_ui:main',
        ],
    },
)
