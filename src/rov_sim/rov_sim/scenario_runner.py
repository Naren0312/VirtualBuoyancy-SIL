# Copyright (c) 2024 Hammerhead ROV Project
# SPDX-License-Identifier: MIT
#
# Scenario Runner — plays repeatable, scripted setpoint/mode sequences
# so tests are deterministic and comparable across code changes.

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from rov_msgs.msg import Setpoint
from std_msgs.msg import UInt8, String

# QoS for /set_mode: TRANSIENT_LOCAL ("latched") so a subscriber that connects
# after the message is sent still receives it. This is critical because the
# mode=1 (TELEOP) message fires once at t=0, and DDS discovery might not yet
# have completed the control_node subscription by then.
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ScenarioRunner(Node):
    """Publishes timed /setpoints and /set_mode from a YAML scenario file."""

    def __init__(self):
        super().__init__('scenario_runner')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('tick_rate_hz', 30.0)

        scenario_path = self.get_parameter('scenario_file').value
        tick_rate = self.get_parameter('tick_rate_hz').value

        if not scenario_path:
            self.get_logger().fatal('No scenario_file parameter provided — exiting')
            raise SystemExit(1)

        # ── Load scenario YAML ───────────────────────────────────────────────
        self.get_logger().info(f'Loading scenario: {scenario_path}')
        with open(scenario_path, 'r') as f:
            data = yaml.safe_load(f)

        scenario = data.get('scenario', data)
        self._name = scenario.get('name', 'unnamed')
        self._steps = scenario.get('steps', [])

        if not self._steps:
            self.get_logger().fatal('Scenario has no steps — exiting')
            raise SystemExit(1)

        # Sort steps by time
        self._steps.sort(key=lambda s: s.get('time', 0.0))

        self.get_logger().info(
            f'Scenario "{self._name}": {len(self._steps)} steps, '
            f'duration {self._steps[-1].get("time", 0.0):.1f}s')

        # ── Publishers ───────────────────────────────────────────────────────
        self._setpoint_pub = self.create_publisher(Setpoint, '/setpoints', 10)
        # TRANSIENT_LOCAL so control_node receives the last mode even if it
        # subscribes after this publishes (fixes VOLATILE one-shot loss).
        self._mode_pub = self.create_publisher(UInt8, '/set_mode', _LATCHED_QOS)
        self._status_pub = self.create_publisher(String, '/scenario_status', 10)

        # ── State ────────────────────────────────────────────────────────────
        self._step_idx = 0
        self._start_time = None
        self._complete = False

        # Current values (carry forward between steps)
        self._current_setpoints = [0.0] * 6
        self._current_mode = None   # None = not yet published

        # ── Timer ────────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / tick_rate, self._tick)

    def _tick(self):
        """Check elapsed time and fire scenario steps."""
        now = self.get_clock().now()

        if self._start_time is None:
            self._start_time = now
            self._publish_status('RUNNING')

        elapsed = (now - self._start_time).nanoseconds / 1e9

        # Fire all steps whose time has arrived
        while (self._step_idx < len(self._steps) and
               elapsed >= self._steps[self._step_idx].get('time', 0.0)):

            step = self._steps[self._step_idx]
            self.get_logger().info(
                f'Step {self._step_idx} @ t={step.get("time", 0.0):.2f}s')

            # Publish mode if specified
            if 'mode' in step:
                mode_msg = UInt8()
                mode_msg.data = int(step['mode'])
                self._mode_pub.publish(mode_msg)
                self._current_mode = int(step['mode'])
                self.get_logger().info(
                    f'  mode → {self._current_mode} '
                    f'({"SURFACE" if self._current_mode == 0 else "TELEOP"})')

            # Publish setpoints if specified
            if 'setpoints' in step:
                sp = step['setpoints']
                for i in range(min(6, len(sp))):
                    self._current_setpoints[i] = float(sp[i])

                sp_msg = Setpoint()
                for i in range(6):
                    sp_msg.setpoints[i] = self._current_setpoints[i]
                self._setpoint_pub.publish(sp_msg)
                self.get_logger().info(
                    f'  setpoints → {self._current_setpoints}')

            self._step_idx += 1

        # Check completion
        if self._step_idx >= len(self._steps) and not self._complete:
            self._complete = True
            self._publish_status('COMPLETE')
            self.get_logger().info(
                f'Scenario "{self._name}" COMPLETE at t={elapsed:.2f}s')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
