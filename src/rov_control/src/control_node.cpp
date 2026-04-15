// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Original authors: Prabin Rath, Rohit Suri, Sonali Agrawal
// Ported to ROS2 Humble for ROV use.

#include "rov_control/control_node.hpp"

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════════

ControlNode::ControlNode() : Node("control_node") {

  // ── Declare and load PID parameters ────────────────────────────────────────
  this->declare_parameter<std::vector<double>>(
      "pid.kp", {0.0, 1.0, 3.0, 0.2, 0.2, 3.0});
  this->declare_parameter<std::vector<double>>(
      "pid.ki", {0.0, 0.024, 0.031, 0.0, 0.0, 0.165});
  this->declare_parameter<std::vector<double>>(
      "pid.kd", {0.0, 1.0, 1.0, 0.0, 0.0, 4.0});

  auto kp_vec = this->get_parameter("pid.kp").as_double_array();
  auto ki_vec = this->get_parameter("pid.ki").as_double_array();
  auto kd_vec = this->get_parameter("pid.kd").as_double_array();

  for (int i = 0; i < DOF_COUNT; i++) {
    kp_[i] = static_cast<float>(kp_vec[i]);
    ki_[i] = static_cast<float>(ki_vec[i]);
    kd_[i] = static_cast<float>(kd_vec[i]);
  }

  // ── Thruster reverse flags ─────────────────────────────────────────────────
  this->declare_parameter<std::vector<bool>>(
      "thruster_reverse", {false, false, false, true, true, false});
  auto rev = this->get_parameter("thruster_reverse").as_bool_array();
  for (int i = 0; i < DOF_COUNT; i++) {
    thruster_reverse_[i] = rev[i];
  }

  // ── Control tuning parameters ──────────────────────────────────────────────
  this->declare_parameter("force_deadband", 0.01);
  this->declare_parameter("surface_depth", 0.22);
  this->declare_parameter("integral_clamp", 2.55);
  this->declare_parameter("pwm_min", 1250);
  this->declare_parameter("pwm_max", 1750);
  this->declare_parameter("pwm_neutral", 1500);
  this->declare_parameter("pitch_tare", 0.0);
  this->declare_parameter("roll_tare", 0.0);

  force_deadband_  = static_cast<float>(this->get_parameter("force_deadband").as_double());
  surface_depth_   = static_cast<float>(this->get_parameter("surface_depth").as_double());
  integral_clamp_  = static_cast<float>(this->get_parameter("integral_clamp").as_double());
  pwm_min_         = this->get_parameter("pwm_min").as_int();
  pwm_max_         = this->get_parameter("pwm_max").as_int();
  pwm_neutral_     = this->get_parameter("pwm_neutral").as_int();
  pitch_tare_      = static_cast<float>(this->get_parameter("pitch_tare").as_double());
  roll_tare_       = static_cast<float>(this->get_parameter("roll_tare").as_double());

  // ── Alpha lookup table (sway coupling) ─────────────────────────────────────
  this->declare_parameter<std::vector<double>>(
      "alpha_table", {0.1, 0.1, 1.1, 0.1, 10.5, 0.3, 1.3, 10.5});
  this->declare_parameter("alpha_default", 0.0);

  auto alpha_vec = this->get_parameter("alpha_table").as_double_array();
  for (size_t i = 0; i < 8 && i < alpha_vec.size(); i++) {
    alpha_table_[i] = static_cast<float>(alpha_vec[i]);
  }
  alpha_default_ = static_cast<float>(this->get_parameter("alpha_default").as_double());

  // ── Surge zero setpoint special PID (preserved from original) ──────────────
  this->declare_parameter("surge_zero_kp", 0.642);
  this->declare_parameter("surge_zero_kd", 0.756);
  this->declare_parameter("surge_zero_ki", 0.007);
  surge_zero_kp_ = static_cast<float>(this->get_parameter("surge_zero_kp").as_double());
  surge_zero_kd_ = static_cast<float>(this->get_parameter("surge_zero_kd").as_double());
  surge_zero_ki_ = static_cast<float>(this->get_parameter("surge_zero_ki").as_double());

  // ── Heave low-pass filter (preserved from original) ────────────────────────
  this->declare_parameter("heave_lpf_threshold", 0.5);
  this->declare_parameter("heave_lpf_alpha", 0.975);
  heave_lpf_threshold_ = static_cast<float>(this->get_parameter("heave_lpf_threshold").as_double());
  heave_lpf_alpha_     = static_cast<float>(this->get_parameter("heave_lpf_alpha").as_double());

  // ── Initialize PID state ───────────────────────────────────────────────────
  reset_pid();

  // ── Create publishers ──────────────────────────────────────────────────────
  thruster_pub_ = this->create_publisher<rov_msgs::msg::ThrusterCommand>(
      "/thruster_command", 10);
  thruster_status_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "/thruster_status", 10);

  // ── Create subscribers ─────────────────────────────────────────────────────
  sensor_sub_ = this->create_subscription<rov_msgs::msg::SensorData>(
      "/sensor_data", 10,
      std::bind(&ControlNode::sensor_data_callback, this, std::placeholders::_1));

  setpoint_sub_ = this->create_subscription<rov_msgs::msg::Setpoint>(
      "/setpoints", 10,
      std::bind(&ControlNode::setpoint_callback, this, std::placeholders::_1));

  pid_gains_sub_ = this->create_subscription<rov_msgs::msg::PIDGains>(
      "/pid_gains", 10,
      std::bind(&ControlNode::pid_gains_callback, this, std::placeholders::_1));

  mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/set_mode", 10,
      std::bind(&ControlNode::mode_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ControlNode initialized — mode: SURFACE");
  RCLCPP_INFO(this->get_logger(), "PID gains loaded: kp=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
              kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensor data callback — runs the full PID loop when in TELEOP mode
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::sensor_data_callback(
    const rov_msgs::msg::SensorData::SharedPtr msg) {

//  if (mode_ != TELEOP) {
//    return;
// }

  // ── Read sensor inputs ─────────────────────────────────────────────────────
  // orientation: [roll, pitch, yaw] in degrees → radians
  in_[ROLL]  = msg->orientation[0] * (M_PI / 180.0f) - roll_tare_;
  in_[PITCH] = msg->orientation[1] * (M_PI / 180.0f) - pitch_tare_;
  in_[YAW]   = msg->orientation[2] * (M_PI / 180.0f);

  // Linear DOFs: surge/sway have no direct sensor feedback (set to 0)
  in_[SURGE] = 0.0f;
  in_[SWAY]  = 0.0f;
  in_[HEAVE] = msg->depth;

  // ── Run PID ────────────────────────────────────────────────────────────────
  compute_pid();
  compute_forces();
  forces_to_pwm();
  publish_thruster_command();
  RCLCPP_INFO(this->get_logger(), "Control callback triggered");
  RCLCPP_INFO(this->get_logger(), "Publishing thruster command");
}

// ═══════════════════════════════════════════════════════════════════════════════
// PID computation — ported from original control.cpp get_output()
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::compute_pid() {

  // ── Error computation ──────────────────────────────────────────────────────
  for (int i = 0; i < 3; i++) {
    // curr_set_[i] is in degrees, in_[i] is in radians
    float setpoint_rad = curr_set_[i] * (M_PI / 180.0f);
    err_[i] = setpoint_rad - in_[i];
    // Angular wrapping via atan2(sin, cos)
    err_[i] = std::atan2(std::sin(err_[i]), std::cos(err_[i]));
    
    // Linear DOFs: Surge, Sway, Heave
    err_[i + 3] = curr_set_[i + 3] - in_[i + 3];
  }

  // ── Derivative, integral with windup clamping ──────────────────────────────
  for (int i = 0; i < DOF_COUNT; i++) {
    err_dot_[i] = err_[i] - perr_[i];
    perr_[i] = err_[i];

    cerr_[i] += err_[i];
    if (cerr_[i] > integral_clamp_)
      cerr_[i] = integral_clamp_;
    else if (cerr_[i] < -integral_clamp_)
      cerr_[i] = -integral_clamp_;

    // Reset integral on setpoint change
    if (pre_setpoint_[i] != curr_set_[i])
      cerr_[i] = 0.0f;
    pre_setpoint_[i] = curr_set_[i];
  }

  // ── PID output ─────────────────────────────────────────────────────────────
  for (int i = 0; i < DOF_COUNT; i++) {
    out_[i] = kp_[i] * err_[i] + kd_[i] * err_dot_[i] + ki_[i] * cerr_[i];
  }

  // ── Surge special case: hardcoded gains when setpoint == 0 ─────────────────
  if (curr_set_[SURGE] == 0.0f) {
    out_[SURGE] = surge_zero_kp_ * err_[SURGE] +
                  surge_zero_kd_ * err_dot_[SURGE] +
                  surge_zero_ki_ * cerr_[SURGE];
  }

  // ── Heave low-pass filter (conditional, preserved from original) ───────────
  if (out_[HEAVE] > heave_lpf_threshold_) {
    out_[HEAVE] = out_[HEAVE] - heave_lpf_alpha_ * (out_[HEAVE] - preout_[HEAVE]);
  }
  preout_[HEAVE] = out_[HEAVE];
}

// ═══════════════════════════════════════════════════════════════════════════════
// Control allocation — maps PID outputs to 6 thruster forces
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::compute_forces() {

  // ── Alpha lookup based on sway setpoint (preserved lookup table) ────────────
  float alpha_tmp = alpha_default_;
  int sway_set = static_cast<int>(curr_set_[SWAY]);
  switch (sway_set) {
    case  2:  alpha_tmp = alpha_table_[0]; break;
    case -2:  alpha_tmp = alpha_table_[1]; break;
    case  3:  alpha_tmp = alpha_table_[2]; break;
    case -3:  alpha_tmp = alpha_table_[3]; break;
    case  5:  alpha_tmp = alpha_table_[4]; break;
    case -5:  alpha_tmp = alpha_table_[5]; break;
    case  10: alpha_tmp = alpha_table_[6]; break;
    case -10: alpha_tmp = alpha_table_[7]; break;
    default:  break;  // use alpha_default_
  }

  // ── Allocation matrix (from original control.cpp lines 176-181) ────────────
  forces_[0] =  out_[HEAVE] - out_[PITCH] - out_[ROLL];
  forces_[1] =  out_[HEAVE] - out_[PITCH] + out_[ROLL];
  forces_[2] =  out_[HEAVE] * 2.0f + 2.0f * out_[PITCH];
  forces_[3] =  out_[YAW]  + out_[SURGE] + alpha_tmp * out_[SWAY];
  forces_[4] = -out_[YAW]  + out_[SURGE] - alpha_tmp * out_[SWAY];
  forces_[5] =  out_[SWAY];
}

// ═══════════════════════════════════════════════════════════════════════════════
// Force → PWM conversion with deadband and clamping
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::forces_to_pwm() {
  for (int i = 0; i < DOF_COUNT; i++) {
    if (forces_[i] >= force_deadband_) {
      forces_[i] = forces_[i] * kForwardScale + kForwardOffset;
    } else if (forces_[i] <= -force_deadband_) {
      forces_[i] = kReverseOffset + forces_[i] * kReverseScale;
    } else {
      forces_[i] = static_cast<float>(pwm_neutral_);
    }

    // Clamp to PWM range
    if (forces_[i] > static_cast<float>(pwm_max_))
      forces_[i] = static_cast<float>(pwm_max_);
    else if (forces_[i] < static_cast<float>(pwm_min_))
      forces_[i] = static_cast<float>(pwm_min_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Publish thruster command
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::publish_thruster_command() {
  rov_msgs::msg::ThrusterCommand cmd;
  for (int i = 0; i < DOF_COUNT; i++) {
    cmd.pwm[i] = static_cast<int16_t>(forces_[i]);
    cmd.reverse[i] = thruster_reverse_[i];
  }
  thruster_pub_->publish(cmd);

  // Publish thruster status = 1 (active)
  std_msgs::msg::Int8 status;
  status.data = 1;
  thruster_status_pub_->publish(status);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setpoint callback — receives operator commands from teleop
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::setpoint_callback(
    const rov_msgs::msg::Setpoint::SharedPtr msg) {
  for (int i = 0; i < DOF_COUNT; i++) {
    curr_set_[i] = msg->setpoints[i];
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// PID gains callback — runtime tuning
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::pid_gains_callback(
    const rov_msgs::msg::PIDGains::SharedPtr msg) {
  for (int i = 0; i < DOF_COUNT; i++) {
    kp_[i] = msg->kp[i];
    ki_[i] = msg->ki[i];
    kd_[i] = msg->kd[i];
  }
  RCLCPP_INFO(this->get_logger(), "PID gains updated");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mode callback — SURFACE / TELEOP
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  auto new_mode = static_cast<Mode>(msg->data);

  if (new_mode == mode_) return;

  switch (new_mode) {
    case SURFACE:
      RCLCPP_INFO(this->get_logger(), "Mode → SURFACE — thrusters disabled");
      reset_pid();
      thruster_active_ = false;

      // Send neutral PWM to all thrusters
      {
        rov_msgs::msg::ThrusterCommand cmd;
        for (int i = 0; i < DOF_COUNT; i++) {
          cmd.pwm[i] = static_cast<int16_t>(pwm_neutral_);
          cmd.reverse[i] = thruster_reverse_[i];
        }
        thruster_pub_->publish(cmd);
      }

      // Publish thruster status = 2 (re-initialize)
      {
        std_msgs::msg::Int8 status;
        status.data = 2;
        thruster_status_pub_->publish(status);
      }
      break;

    case TELEOP:
      RCLCPP_INFO(this->get_logger(), "Mode → TELEOP — thrusters enabled");
      reset_pid();

      // Set initial heave setpoint to surface depth
      curr_set_[ROLL]  = 0.0f;
      curr_set_[PITCH] = 0.0f;
      curr_set_[SURGE] = 0.0f;
      curr_set_[SWAY]  = 0.0f;
      curr_set_[HEAVE] = surface_depth_;

      thruster_active_ = true;

      // Re-initialize thrusters
      {
        std_msgs::msg::Int8 status;
        status.data = 2;
        thruster_status_pub_->publish(status);
      }
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown mode %d — reverting to SURFACE",
                  msg->data);
      reset_pid();
      thruster_active_ = false;
      new_mode = SURFACE;
      break;
  }

  mode_ = new_mode;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Reset all PID state
// ═══════════════════════════════════════════════════════════════════════════════

void ControlNode::reset_pid() {
  in_.fill(0.0f);
  err_.fill(0.0f);
  err_dot_.fill(0.0f);
  perr_.fill(0.0f);
  cerr_.fill(0.0f);
  out_.fill(0.0f);
  preout_.fill(0.0f);
  pre_setpoint_.fill(0.0f);
  forces_.fill(0.0f);
}
