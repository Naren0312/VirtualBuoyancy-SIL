// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Original author: (Teleop.cpp)
// Ported to ROS2 Humble for ROV joystick teleoperation.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rov_msgs/msg/setpoint.hpp"

#include <array>

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode();
  ~TeleopNode() override = default;

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<rov_msgs::msg::Setpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;

  std::array<float, 6> setpoints_{};  // [roll, pitch, yaw, surge, sway, heave]

  // ── Configurable axis/button mappings ──────────────────────────────────────
  int axis_yaw_coarse_;    // joystick axis for coarse yaw
  int axis_yaw_fine_;      // joystick axis for fine yaw
  int btn_surge_fwd_;      // button: surge forward
  int btn_surge_bwd_;      // button: surge backward
  int btn_sway_left_;      // button: sway left
  int btn_sway_right_;     // button: sway right
  int btn_heave_up_;       // button: heave up (deeper)
  int btn_heave_down_;     // button: heave down (shallower)
  int btn_pitch_up_;       // button: pitch up
  int btn_pitch_down_;     // button: pitch down
  int btn_stop_motion_;    // button: stop surge/sway
  int btn_emergency_stop_; // button: full emergency stop (SURFACE mode)

  // ── Limits ─────────────────────────────────────────────────────────────────
  float surge_limit_;
  float sway_limit_;
  float heave_min_;
  float heave_max_;
  float yaw_limit_;
  float yaw_coarse_step_;
  float yaw_fine_step_;
  float heave_step_;
  float pitch_step_;

  float initial_heave_;
};
