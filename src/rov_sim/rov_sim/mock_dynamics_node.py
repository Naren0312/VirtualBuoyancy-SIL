# Copyright (c) 2024 Hammerhead ROV Project
# SPDX-License-Identifier: MIT
#
# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  TEMPORARY SCAFFOLDING — Unity will replace this node.                     ║
# ║                                                                            ║
# ║  This node closes the control loop in software by subscribing to           ║
# ║  /thruster_command and publishing /depth + /imu_data.  It uses a crude     ║
# ║  6-DOF mass-damping model — NOT accurate hydrodynamics.  The goal is a     ║
# ║  stable, sane loop for PID validation, not high-fidelity simulation.       ║
# ║                                                                            ║
# ║  Once Unity is online, swap to sim_backend:=unity in the launch file and   ║
# ║  this node is no longer needed.                                            ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

import math

import rclpy
from rclpy.node import Node

from rov_msgs.msg import Depth, IMUData, ThrusterCommand


# ── PWM ↔ force constants (mirrored from control_node.hpp) ──────────────────
_FWD_SCALE = 370.0 / 2.36   # ≈ 156.78
_FWD_OFFSET = 1530
_REV_SCALE = 370.0 / 1.85   # ≈ 200.00
_REV_OFFSET = 1470

# ── DOF indices (matches control_node.hpp enum) ─────────────────────────────
ROLL, PITCH, YAW, SURGE, SWAY, HEAVE = range(6)


class MockDynamicsNode(Node):
    """Temporary stand-in for Unity — crude 6-DOF physics to close the loop."""

    def __init__(self):
        super().__init__('mock_dynamics_node')

        # ── Tunable simulation parameters ────────────────────────────────────
        self.declare_parameter('sim_rate_hz', 30.0)
        self.declare_parameter('mass', 12.0)           # kg  (linear DOFs)
        self.declare_parameter('inertia', 2.0)         # kg·m²  (angular DOFs)
        self.declare_parameter('damping_linear', 8.0)  # N·s/m
        self.declare_parameter('damping_angular', 4.0)  # N·m·s/rad
        self.declare_parameter('buoyancy_k', 3.0)      # N/m  heave restoring
        self.declare_parameter('righting_k', 1.5)      # N·m/rad  roll/pitch restoring
        self.declare_parameter('initial_depth', 0.22)   # m  (matches surface_depth)

        sim_rate = self.get_parameter('sim_rate_hz').value
        self._mass = self.get_parameter('mass').value
        self._inertia = self.get_parameter('inertia').value
        self._damp_lin = self.get_parameter('damping_linear').value
        self._damp_ang = self.get_parameter('damping_angular').value
        self._buoy_k = self.get_parameter('buoyancy_k').value
        self._right_k = self.get_parameter('righting_k').value
        initial_depth = self.get_parameter('initial_depth').value

        self._dt = 1.0 / sim_rate

        # ── State vectors: [roll, pitch, yaw, surge, sway, heave] ───────────
        self._pos = [0.0] * 6   # rad for angular, m for linear
        self._vel = [0.0] * 6
        self._pos[HEAVE] = initial_depth

        # ── Latest thruster command ──────────────────────────────────────────
        self._last_pwm = [1500] * 6
        self._last_reverse = [False] * 6

        # ── Publishers ───────────────────────────────────────────────────────
        self._depth_pub = self.create_publisher(Depth, '/depth', 10)
        self._imu_pub = self.create_publisher(IMUData, '/imu_data', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(
            ThrusterCommand, '/thruster_command',
            self._thruster_cb, 10)

        # ── Simulation timer ─────────────────────────────────────────────────
        self._timer = self.create_timer(self._dt, self._sim_step)

        self.get_logger().info(
            f'MockDynamicsNode started — {sim_rate:.0f} Hz, '
            f'mass={self._mass}, dt={self._dt:.4f}s  '
            f'[TEMPORARY — Unity replaces this]')

    # ─────────────────────────────────────────────────────────────────────────
    # Thruster command callback — just store, physics runs on the timer
    # ─────────────────────────────────────────────────────────────────────────

    def _thruster_cb(self, msg: ThrusterCommand):
        self._last_pwm = list(msg.pwm)
        self._last_reverse = list(msg.reverse)

    # ─────────────────────────────────────────────────────────────────────────
    # PWM → per-thruster force (inverse of control_node's forces_to_pwm)
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _pwm_to_force(pwm: int, reverse: bool) -> float:
        """Invert the force→PWM mapping from control_node.hpp."""
        if pwm > _FWD_OFFSET:
            force = (pwm - _FWD_OFFSET) / _FWD_SCALE
        elif pwm < _REV_OFFSET:
            force = (pwm - _REV_OFFSET) / _REV_SCALE
        else:
            force = 0.0

        # Honour the per-thruster reverse flag
        if reverse:
            force = -force

        return force

    # ─────────────────────────────────────────────────────────────────────────
    # Inverse allocation:  6 thruster forces → 6 DOF demands
    #
    # Forward allocation (from control_node.cpp):
    #   f[0] =  heave - pitch - roll          (vertical 1)
    #   f[1] =  heave - pitch + roll          (vertical 2)
    #   f[2] =  heave*2 + pitch*2             (vertical 3)
    #   f[3] =  yaw + surge + α·sway          (horizontal 1)
    #   f[4] = -yaw + surge - α·sway          (horizontal 2)
    #   f[5] =  sway                          (horizontal 3)
    #
    # Analytical inverse (α assumed ≈ 0 for the mock):
    #   roll  = (f1 - f0) / 2
    #   heave = (f0 + f1 + f2) / 4
    #   pitch = (f2 - f0 - f1) / 4
    #   yaw   = (f3 - f4) / 2
    #   surge = (f3 + f4) / 2
    #   sway  = f5
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _inverse_allocation(forces):
        """Map 6 thruster forces back to 6 DOF demands."""
        f = forces
        dof = [0.0] * 6
        dof[ROLL]  = (f[1] - f[0]) / 2.0
        dof[PITCH] = (f[2] - f[0] - f[1]) / 4.0
        dof[YAW]   = (f[3] - f[4]) / 2.0
        dof[SURGE] = (f[3] + f[4]) / 2.0
        dof[SWAY]  = f[5]
        dof[HEAVE] = (f[0] + f[1] + f[2]) / 4.0
        return dof

    # ─────────────────────────────────────────────────────────────────────────
    # Simulation step — runs at sim_rate_hz
    # ─────────────────────────────────────────────────────────────────────────

    def _sim_step(self):
        # 1) PWM → per-thruster force
        thruster_forces = [
            self._pwm_to_force(self._last_pwm[i], self._last_reverse[i])
            for i in range(6)
        ]

        # 2) Inverse allocation → DOF demands
        dof_forces = self._inverse_allocation(thruster_forces)

        # 3) Add restoring forces
        #    - Buoyancy on heave: spring pulling toward 0 (surface)
        #    - Righting moment on roll and pitch
        dof_forces[HEAVE] -= self._buoy_k * (self._pos[HEAVE] - 0.0)
        dof_forces[ROLL]  -= self._right_k * self._pos[ROLL]
        dof_forces[PITCH] -= self._right_k * self._pos[PITCH]

        # 4) Integrate each DOF: mass-damping model
        for i in range(6):
            if i in (ROLL, PITCH, YAW):
                m = self._inertia
                d = self._damp_ang
            else:
                m = self._mass
                d = self._damp_lin

            accel = (dof_forces[i] / m) - (d / m) * self._vel[i]
            self._vel[i] += accel * self._dt
            self._pos[i] += self._vel[i] * self._dt

        # 5) Clamp depth ≥ 0  (can't fly out of the water)
        if self._pos[HEAVE] < 0.0:
            self._pos[HEAVE] = 0.0
            self._vel[HEAVE] = max(self._vel[HEAVE], 0.0)

        # 6) Wrap yaw to [-π, π]
        self._pos[YAW] = math.atan2(
            math.sin(self._pos[YAW]),
            math.cos(self._pos[YAW]))

        # 7) Publish /depth and /imu_data with matching timestamps
        now = self.get_clock().now().to_msg()

        depth_msg = Depth()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'base_link'
        depth_msg.depth = float(self._pos[HEAVE])
        self._depth_pub.publish(depth_msg)

        imu_msg = IMUData()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'base_link'
        # orientation: [roll, pitch, yaw] in DEGREES (matches real IMU)
        imu_msg.orientation[0] = float(math.degrees(self._pos[ROLL]))
        imu_msg.orientation[1] = float(math.degrees(self._pos[PITCH]))
        imu_msg.orientation[2] = float(math.degrees(self._pos[YAW]))
        # acceleration: zero for now (crude mock)
        imu_msg.acceleration[0] = 0.0
        imu_msg.acceleration[1] = 0.0
        imu_msg.acceleration[2] = 0.0
        self._imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockDynamicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
