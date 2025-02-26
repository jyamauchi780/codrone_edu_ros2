from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'codrone_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools',
                      'codrone_msgs',
                      ],
    zip_safe=True,
    maintainer='Junya Yamauchi',
    maintainer_email='yamauchi780@gmail.com',
    description='ROS2 driver for Robolink CoDrone Edu',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'codrone_driver = codrone_ros2_driver.codrone_driver:main',
            'ident_codrone_driver = codrone_ros2_driver.ident_codrone_driver:main',
            'ident_input_generator_mocap = codrone_ros2_driver.ident_input_generator_mocap:main',
            'velocity_observer = codrone_ros2_driver.velocity_observer:main',
            'velocity_controller = codrone_ros2_driver.velocity_controller:main',
            'velocity_power_convert_checker = codrone_ros2_driver.velocity_power_convert_checker:main',
            'generate_cmd_vel_from_codrone_joy = codrone_ros2_driver.generate_cmd_vel_from_codrone_joy:main',
        ],
    },
)
