from setuptools import setup

package_name = 'next_ros2ws_tools'

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
    description='Tools/experiments for next_ros2ws (may use heavy deps like OpenCV/numpy).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_reloc = next_ros2ws_tools.auto_reloc:main',
        ], 
    },
)

