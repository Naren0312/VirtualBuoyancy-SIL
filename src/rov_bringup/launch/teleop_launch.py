# Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
# SPDX-License-Identifier: MIT
#
# Operator station launch file — joystick + teleop node.
# Run this on the topside computer connected to the joystick.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rov_bringup'),
        'config',
        'rov_params.yaml'
    )

    return LaunchDescription([

        # ── Joy Node: reads /dev/input/js0 → publishes /joy ──────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
        ),

        # ── Teleop Node: /joy → /setpoints + /set_mode ──────────────────────
        Node(
            package='rov_control',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[config],
        ),
    ])
