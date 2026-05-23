# Copyright (c) 2024 Hammerhead ROV Project
# SPDX-License-Identifier: MIT
#
# SIL Simulator launch file
#
# Launches the real control stack (control_node, sensor_sync_node) with either:
#   - mock_dynamics_node  (sim_backend:=mock, default — tests without Unity)
#   - ros_tcp_endpoint    (sim_backend:=unity — for Unity on the RTX machine)
#
# Does NOT launch serial_bridge_node or thruster_driver_node (those are the
# physical hardware that the simulator replaces).
#
# Usage:
#   ros2 launch rov_sim rov_sim_launch.py                         # mock + teleop
#   ros2 launch rov_sim rov_sim_launch.py sim_backend:=unity      # Unity bridge
#   ros2 launch rov_sim rov_sim_launch.py use_teleop:=false \
#       scenario_file:=/path/to/scenario.yaml                     # automated test

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _generate_nodes(context):
    """Resolve launch args and return the appropriate node list."""

    # ── Resolve launch arguments ─────────────────────────────────────────────
    sim_backend = LaunchConfiguration('sim_backend').perform(context)
    use_teleop = LaunchConfiguration('use_teleop').perform(context).lower()
    scenario_file = LaunchConfiguration('scenario_file').perform(context)

    # ── Shared parameter file (same as real rov_launch.py) ───────────────────
    config = os.path.join(
        get_package_share_directory('rov_bringup'),
        'config',
        'rov_params.yaml'
    )

    nodes = []

    # ── Sensor Synchronizer (unchanged from real vehicle) ────────────────────
    nodes.append(Node(
        package='rov_hardware',
        executable='sensor_sync_node',
        name='sensor_sync_node',
        output='screen',
        parameters=[config],
    ))

    # ── Control Node (the device under test — unchanged) ─────────────────────
    nodes.append(Node(
        package='rov_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[config],
    ))

    # ── Input source: teleop OR scenario runner ──────────────────────────────
    if use_teleop in ('true', '1', 'yes'):
        nodes.append(Node(
            package='rov_control',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[config],
        ))
    else:
        # Automated test — use scenario_runner
        if not scenario_file:
            # Default to the depth_hold scenario
            scenario_file = os.path.join(
                get_package_share_directory('rov_sim'),
                'config', 'scenarios', 'depth_hold.yaml'
            )
        nodes.append(Node(
            package='rov_sim',
            executable='scenario_runner',
            name='scenario_runner',
            output='screen',
            parameters=[{'scenario_file': scenario_file}],
        ))

    # ── Sim backend: mock OR Unity TCP bridge ────────────────────────────────
    if sim_backend == 'mock':
        nodes.append(Node(
            package='rov_sim',
            executable='mock_dynamics_node',
            name='mock_dynamics_node',
            output='screen',
            parameters=[config],
        ))
    elif sim_backend == 'unity':
        nodes.append(Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            output='screen',
            parameters=[{'ROS_IP': '0.0.0.0'}],
        ))
    else:
        raise ValueError(
            f"Unknown sim_backend '{sim_backend}' — use 'mock' or 'unity'")

    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'sim_backend', default_value='mock',
            description="Simulation backend: 'mock' (Python stand-in) or "
                        "'unity' (TCP bridge to Unity)"),

        DeclareLaunchArgument(
            'use_teleop', default_value='true',
            description="'true' = launch teleop_node for joystick, "
                        "'false' = launch scenario_runner for automated tests"),

        DeclareLaunchArgument(
            'scenario_file', default_value='',
            description='Path to scenario YAML (used when use_teleop:=false)'),

        OpaqueFunction(function=_generate_nodes),
    ])
