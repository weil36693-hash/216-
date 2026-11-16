from setuptools import find_packages, setup


package_name = 'leap_fleet'
fleet_config = '../../config/fleet.json'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'leap_fleet': ['web_static/*.html', 'web_static/*.css', 'web_static/*.js']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/leap_fleet']),
        ('share/leap_fleet', ['package.xml', 'README.txt']),
        ('share/leap_fleet/config', [fleet_config]),
        ('share/leap_fleet/examples', ['examples/fleet_trajectory_template.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-car',
    maintainer_email='chuyun@example.com',
    description='Multi-domain differential-drive commands and read-only mocap telemetry.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['drive = leap_fleet.cli:main',
                                     'console = leap_fleet.web_console:main']},
)
