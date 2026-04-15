# Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
# SPDX-License-Identifier: MIT
#
# Main ROV launch file — starts all 4 core nodes on the vehicle.

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

        # ── Serial Bridge: Arduino ↔ ROS2 ────────────────────────────────────
        Node(
            package='rov_hardware',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Sensor Synchronizer: fuses /depth + /imu_data → /sensor_data ─────
        Node(
            package='rov_hardware',
            executable='sensor_sync_node',
            name='sensor_sync_node',
            output='screen',
            parameters=[config],
        ),

        # ── Control Node: PID + control allocation ───────────────────────────
        Node(
            package='rov_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[config],
        ),

        # ── Thruster Driver: PWM → Pololu Maestro ────────────────────────────
        Node(
            package='rov_hardware',
            executable='thruster_driver_node',
            name='thruster_driver_node',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
