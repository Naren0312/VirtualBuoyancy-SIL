// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Ported from mission_stack/joystick/src/Teleop.cpp
// Simplified for direct ROV joystick control.

#include "rov_control/teleop_node.hpp"

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════════

TeleopNode::TeleopNode() : Node("teleop_node") {

  // ── Axis/button mapping parameters ─────────────────────────────────────────
  this->declare_parameter("axis_yaw_coarse", 0);
  this->declare_parameter("axis_yaw_fine", 3);
  this->declare_parameter("btn_surge_fwd", 0);
  this->declare_parameter("btn_surge_bwd", 2);
  this->declare_parameter("btn_sway_left", 1);
  this->declare_parameter("btn_sway_right", 3);
  this->declare_parameter("btn_heave_up", 7);
  this->declare_parameter("btn_heave_down", 5);
  this->declare_parameter("btn_pitch_up", 4);
  this->declare_parameter("btn_pitch_down", 6);
  this->declare_parameter("btn_stop_motion", 9);
  this->declare_parameter("btn_emergency_stop", 8);

  axis_yaw_coarse_    = this->get_parameter("axis_yaw_coarse").as_int();
  axis_yaw_fine_      = this->get_parameter("axis_yaw_fine").as_int();
  btn_surge_fwd_      = this->get_parameter("btn_surge_fwd").as_int();
  btn_surge_bwd_      = this->get_parameter("btn_surge_bwd").as_int();
  btn_sway_left_      = this->get_parameter("btn_sway_left").as_int();
  btn_sway_right_     = this->get_parameter("btn_sway_right").as_int();
  btn_heave_up_       = this->get_parameter("btn_heave_up").as_int();
  btn_heave_down_     = this->get_parameter("btn_heave_down").as_int();
  btn_pitch_up_       = this->get_parameter("btn_pitch_up").as_int();
  btn_pitch_down_     = this->get_parameter("btn_pitch_down").as_int();
  btn_stop_motion_    = this->get_parameter("btn_stop_motion").as_int();
  btn_emergency_stop_ = this->get_parameter("btn_emergency_stop").as_int();

  // ── Limit parameters ──────────────────────────────────────────────────────
  this->declare_parameter("surge_limit", 5.0);
  this->declare_parameter("sway_limit", 5.0);
  this->declare_parameter("heave_min", 0.0);
  this->declare_parameter("heave_max", 10.0);
  this->declare_parameter("yaw_limit", 180.0);
  this->declare_parameter("yaw_coarse_step", 10.0);
  this->declare_parameter("yaw_fine_step", 1.0);
  this->declare_parameter("heave_step", 0.1);
  this->declare_parameter("pitch_step", 10.0);
  this->declare_parameter("initial_heave", 0.22);

  surge_limit_     = static_cast<float>(this->get_parameter("surge_limit").as_double());
  sway_limit_      = static_cast<float>(this->get_parameter("sway_limit").as_double());
  heave_min_       = static_cast<float>(this->get_parameter("heave_min").as_double());
  heave_max_       = static_cast<float>(this->get_parameter("heave_max").as_double());
  yaw_limit_       = static_cast<float>(this->get_parameter("yaw_limit").as_double());
  yaw_coarse_step_ = static_cast<float>(this->get_parameter("yaw_coarse_step").as_double());
  yaw_fine_step_   = static_cast<float>(this->get_parameter("yaw_fine_step").as_double());
  heave_step_      = static_cast<float>(this->get_parameter("heave_step").as_double());
  pitch_step_      = static_cast<float>(this->get_parameter("pitch_step").as_double());
  initial_heave_   = static_cast<float>(this->get_parameter("initial_heave").as_double());

  // ── Initialize setpoints ───────────────────────────────────────────────────
  setpoints_.fill(0.0f);
  setpoints_[5] = initial_heave_;  // heave = initial depth

  // ── Create publishers ──────────────────────────────────────────────────────
  setpoint_pub_ = this->create_publisher<rov_msgs::msg::Setpoint>("/setpoints", 10);
  mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/set_mode", 10);

  // ── Subscribe to joystick ──────────────────────────────────────────────────
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TeleopNode initialized — waiting for joystick input");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Joystick callback — maps axes/buttons to setpoints
// Ported from original Teleop.cpp joyCallback
// ═══════════════════════════════════════════════════════════════════════════════

void TeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {

  // Safety check: ensure enough axes/buttons
  if (static_cast<int>(joy->axes.size()) <= std::max(axis_yaw_coarse_, axis_yaw_fine_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Joystick has insufficient axes");
    return;
  }

  // ── Yaw control (coarse + fine axes) ───────────────────────────────────────
  if (joy->axes[axis_yaw_coarse_] > 0.5f)
    setpoints_[2] -= yaw_coarse_step_;  // YAW index = 2
  else if (joy->axes[axis_yaw_coarse_] < -0.5f)
    setpoints_[2] += yaw_coarse_step_;

  if (joy->axes[axis_yaw_fine_] > 0.5f)
    setpoints_[2] += yaw_fine_step_;
  else if (joy->axes[axis_yaw_fine_] < -0.5f)
    setpoints_[2] -= yaw_fine_step_;

  // ── Surge control (buttons) ────────────────────────────────────────────────
  if (btn_surge_fwd_ < static_cast<int>(joy->buttons.size()))
    setpoints_[3] += joy->buttons[btn_surge_fwd_];   // SURGE index = 3
  if (btn_surge_bwd_ < static_cast<int>(joy->buttons.size()))
    setpoints_[3] -= joy->buttons[btn_surge_bwd_];

  // ── Sway control (buttons) ─────────────────────────────────────────────────
  if (btn_sway_left_ < static_cast<int>(joy->buttons.size()))
    setpoints_[4] += joy->buttons[btn_sway_left_];   // SWAY index = 4
  if (btn_sway_right_ < static_cast<int>(joy->buttons.size()))
    setpoints_[4] -= joy->buttons[btn_sway_right_];

  // ── Heave control (buttons — step-based depth change) ──────────────────────
  if (btn_heave_up_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_heave_up_])
    setpoints_[5] += heave_step_;   // HEAVE index = 5
  if (btn_heave_down_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_heave_down_])
    setpoints_[5] -= heave_step_;

  // ── Pitch control (buttons) ────────────────────────────────────────────────
  if (btn_pitch_up_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_pitch_up_])
    setpoints_[1] += pitch_step_;   // PITCH index = 1
  if (btn_pitch_down_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_pitch_down_])
    setpoints_[1] -= pitch_step_;

  // ── Clamp all setpoints ────────────────────────────────────────────────────
  // Surge
  if (setpoints_[3] > surge_limit_) setpoints_[3] = surge_limit_;
  else if (setpoints_[3] < -surge_limit_) setpoints_[3] = -surge_limit_;

  // Sway
  if (setpoints_[4] > sway_limit_) setpoints_[4] = sway_limit_;
  else if (setpoints_[4] < -sway_limit_) setpoints_[4] = -sway_limit_;

  // Heave
  if (setpoints_[5] > heave_max_) setpoints_[5] = heave_max_;
  else if (setpoints_[5] < heave_min_) setpoints_[5] = heave_min_;

  // Yaw
  if (setpoints_[2] > yaw_limit_) setpoints_[2] = yaw_limit_;
  else if (setpoints_[2] < -yaw_limit_) setpoints_[2] = -yaw_limit_;

  // Pitch
  if (setpoints_[1] > yaw_limit_) setpoints_[1] = yaw_limit_;
  else if (setpoints_[1] < -yaw_limit_) setpoints_[1] = -yaw_limit_;

  // ── Stop motion button ─────────────────────────────────────────────────────
  if (btn_stop_motion_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_stop_motion_]) {
    setpoints_[3] = 0.0f;  // stop surge
    setpoints_[4] = 0.0f;  // stop sway
  }

  // ── Emergency stop button → SURFACE mode ───────────────────────────────────
  if (btn_emergency_stop_ < static_cast<int>(joy->buttons.size()) && joy->buttons[btn_emergency_stop_]) {
    setpoints_.fill(0.0f);
    setpoints_[5] = initial_heave_;

    std_msgs::msg::UInt8 mode_msg;
    mode_msg.data = 0;  // SURFACE
    mode_pub_->publish(mode_msg);

    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP — mode set to SURFACE");
  }

  // ── Publish setpoints ──────────────────────────────────────────────────────
  rov_msgs::msg::Setpoint sp_msg;
  for (int i = 0; i < 6; i++) {
    sp_msg.setpoints[i] = setpoints_[i];
  }
  setpoint_pub_->publish(sp_msg);
}
