// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

#include "rov_msgs/msg/thruster_command.hpp"
#include "rov_msgs/msg/sensor_data.hpp"
#include "rov_msgs/msg/pid_gains.hpp"
#include "rov_msgs/msg/setpoint.hpp"

#include <array>
#include <cmath>

class ControlNode : public rclcpp::Node {
public:
  ControlNode();
  ~ControlNode() override = default;

private:
  // ── DOF indices ──────────────────────────────────────────────────────────
  enum DOF { ROLL = 0, PITCH, YAW, SURGE, SWAY, HEAVE, DOF_COUNT = 6 };

  // ── Operating modes ──────────────────────────────────────────────────────
  enum Mode : uint8_t { SURFACE = 0, TELEOP = 1 };

  // ── Callbacks ────────────────────────────────────────────────────────────
  void sensor_data_callback(const rov_msgs::msg::SensorData::SharedPtr msg);
  void setpoint_callback(const rov_msgs::msg::Setpoint::SharedPtr msg);
  void pid_gains_callback(const rov_msgs::msg::PIDGains::SharedPtr msg);
  void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);

  // ── Helpers ──────────────────────────────────────────────────────────────
  void reset_pid();
  void compute_pid();
  void compute_forces();
  void forces_to_pwm();
  void publish_thruster_command();

  // ── Subscribers ──────────────────────────────────────────────────────────
  rclcpp::Subscription<rov_msgs::msg::SensorData>::SharedPtr sensor_sub_;
  rclcpp::Subscription<rov_msgs::msg::Setpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<rov_msgs::msg::PIDGains>::SharedPtr pid_gains_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;

  // ── Publishers ───────────────────────────────────────────────────────────
  rclcpp::Publisher<rov_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr thruster_status_pub_;

  // ── PID state arrays ─────────────────────────────────────────────────────
  std::array<float, DOF_COUNT> kp_{};
  std::array<float, DOF_COUNT> ki_{};
  std::array<float, DOF_COUNT> kd_{};

  std::array<float, DOF_COUNT> in_{};          // current sensor input
  std::array<float, DOF_COUNT> err_{};         // current error
  std::array<float, DOF_COUNT> err_dot_{};     // derivative of error
  std::array<float, DOF_COUNT> perr_{};        // previous error
  std::array<float, DOF_COUNT> cerr_{};        // cumulative (integral) error
  std::array<float, DOF_COUNT> out_{};         // PID output
  std::array<float, DOF_COUNT> preout_{};      // previous PID output
  std::array<float, DOF_COUNT> pre_setpoint_{}; // previous setpoint (for integral reset)
  std::array<float, DOF_COUNT> curr_set_{};    // current setpoints
  std::array<float, DOF_COUNT> forces_{};      // force allocation output

  // ── Alpha lookup table (sway coupling) ───────────────────────────────────
  // Indexed by sway setpoint value: ±2, ±3, ±5, ±10
  float alpha_table_[8]{};
  float alpha_default_{0.0f};

  // ── Thruster configuration ───────────────────────────────────────────────
  std::array<bool, DOF_COUNT> thruster_reverse_{};

  // ── Control parameters ───────────────────────────────────────────────────
  float force_deadband_{0.01f};
  float surface_depth_{0.22f};
  float integral_clamp_{2.55f};
  int pwm_min_{1250};
  int pwm_max_{1750};
  int pwm_neutral_{1500};
  float pitch_tare_{0.0f};
  float roll_tare_{0.0f};

  // Original force→PWM conversion constants
  static constexpr float kForwardScale  = 370.0f / 2.36f;
  static constexpr float kReverseScale  = 370.0f / 1.85f;
  static constexpr int   kForwardOffset = 1530;
  static constexpr int   kReverseOffset = 1470;

  // ── Mode state ───────────────────────────────────────────────────────────
  Mode mode_{SURFACE};
  bool thruster_active_{false};

  // ── Surge special-case PID gains (preserved from original) ───────────────
  float surge_zero_kp_{0.642f};
  float surge_zero_kd_{0.756f};
  float surge_zero_ki_{0.007f};

  // ── Heave low-pass filter threshold (preserved from original) ────────────
  float heave_lpf_threshold_{0.5f};
  float heave_lpf_alpha_{0.975f};
};
