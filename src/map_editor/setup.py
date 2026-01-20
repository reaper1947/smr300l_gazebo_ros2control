from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'map_editor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Web templates and static files
        (os.path.join('share', package_name, 'web', 'templates'),
            glob('map_editor/web/templates/*.html')),
        (os.path.join('share', package_name, 'web', 'static', 'js'),
            glob('map_editor/web/static/js/*.js')),
        (os.path.join('share', package_name, 'web', 'static', 'css'),
            glob('map_editor/web/static/css/*.css')),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors', 'pillow', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='aun',
    maintainer_email='s6403014621116@email.kmutnb.ac.th',
    description='Professional map editing tools for warehouse robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_editor_server = map_editor.map_editor_server:main',
        ],
    },
)
