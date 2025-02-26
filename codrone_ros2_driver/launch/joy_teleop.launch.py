# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.events import Shutdown


def generate_launch_description():

    declare_robotdev = DeclareLaunchArgument(
        'robot_dev', default_value='CoDrone1',
        description='Device file for Robot'
    )

    joy_config = os.path.join(
        get_package_share_directory('codrone_ros2_driver'),
        'config',
        'codrone_joy.yml'
    )

    codrone_driver_node = Node(
        package='codrone_ros2_driver',
        namespace='',
        name='codrone_driver',
        executable='codrone_driver',
        parameters=[joy_config, {'robot_dev': LaunchConfiguration('robot_dev')}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_robotdev)
    ld.add_action(codrone_driver_node)

    print(LaunchIntrospector().format_launch_description(ld))

    return ld
