from setuptools import find_packages, setup

package_name = 'zone_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web/templates', 
            ['zone_nav/web/templates/index.html', 'zone_nav/web/templates/map_editor.html']),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors', 'pillow', 'pyyaml', 'psutil'],
    zip_safe=True,
    maintainer='aun',
    maintainer_email='s6403014621116@email.kmutnb.ac.th',
    description='Zone navigation package with custom service interfaces',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_manager = zone_nav.zone_manager:main',
            'zone_web_ui = zone_nav.zone_web_ui:main',
            'mode_switcher = zone_nav.mode_switcher:main',
            'safety_controller = zone_nav.safety_controller:main',
            'scan_merger = zone_nav.scan_merger:main',
            'zone_publisher = zone_nav.keepout_zone_publisher:main',
            'keepout_zone_publisher = zone_nav.keepout_zone_publisher:main',
            'map_manager = zone_nav.map_manager:main',
            'auto_reloc = zone_nav.auto_reloc:main',
        ],
    },
)
