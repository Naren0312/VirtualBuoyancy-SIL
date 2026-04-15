// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Ported from hardware_stack/thruster_driver/src/thruster_driver.cpp
// Writes Pololu Maestro serial commands to control ESCs.

#include "rov_hardware/thruster_driver_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor — opens serial port and configures for raw I/O
// ═══════════════════════════════════════════════════════════════════════════════

ThrusterDriverNode::ThrusterDriverNode() : Node("thruster_driver_node") {

  // ── Parameters ─────────────────────────────────────────────────────────────
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter<std::vector<long int>>(
      "port_mapping", {0, 10, 4, 2, 8, 6});

  serial_port_ = this->get_parameter("serial_port").as_string();
  auto pm = this->get_parameter("port_mapping").as_integer_array();
  for (int i = 0; i < 6 && i < static_cast<int>(pm.size()); i++) {
    port_mapping_[i] = static_cast<int>(pm[i]);
  }

  // ── Open serial device ─────────────────────────────────────────────────────
  fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1) {
    RCLCPP_ERROR(this->get_logger(), "Could not open serial device: %s",
                 serial_port_.c_str());
    // Gracefully continue instead of throwing. Messages to fd_ == -1 will fail safely.
  } else {
    // Configure for raw serial (from original thruster_driver.cpp)
    struct termios options;
    tcgetattr(fd_, &options);
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tcsetattr(fd_, TCSANOW, &options);

    RCLCPP_INFO(this->get_logger(), "Maestro serial port opened: %s",
                serial_port_.c_str());
  }



  // ── Initialize breach timer ────────────────────────────────────────────────
  last_breach_time_ = std::chrono::steady_clock::now();

  // ── Subscribers ────────────────────────────────────────────────────────────
  cmd_sub_ = this->create_subscription<rov_msgs::msg::ThrusterCommand>(
      "/thruster_command", 10,
      std::bind(&ThrusterDriverNode::command_callback, this,
                std::placeholders::_1));

  breach_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/breach_command", 10,
      std::bind(&ThrusterDriverNode::breach_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ThrusterDriverNode initialized — ports: [%d,%d,%d,%d,%d,%d]",
              port_mapping_[0], port_mapping_[1], port_mapping_[2],
              port_mapping_[3], port_mapping_[4], port_mapping_[5]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Destructor — close serial port
// ═══════════════════════════════════════════════════════════════════════════════

ThrusterDriverNode::~ThrusterDriverNode() {
  if (fd_ != -1) {
    write_neutral_to_all();
    close(fd_);
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Write a single Pololu Maestro command
// Protocol: 0x84, channel, target_low7, target_high7
// ═══════════════════════════════════════════════════════════════════════════════

void ThrusterDriverNode::write_to_maestro(int port_num, uint16_t target) {
  target = target * 4;  // Maestro uses quarter-microseconds
  unsigned char command[] = {
      0x84,
      static_cast<unsigned char>(port_num),
      static_cast<unsigned char>(target & 0x7F),
      static_cast<unsigned char>((target >> 7) & 0x7F)};

  if (write(fd_, command, sizeof(command)) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error writing to thruster port %d",
                 port_num);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Set all thrusters to neutral (1500 µs)
// ═══════════════════════════════════════════════════════════════════════════════

void ThrusterDriverNode::write_neutral_to_all() {
  for (int i = 0; i < 6; i++) {
    write_to_maestro(port_mapping_[i], 1500);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Breach callback — emergency stop, send neutral to all
// ═══════════════════════════════════════════════════════════════════════════════

void ThrusterDriverNode::breach_callback(
    const std_msgs::msg::Bool::SharedPtr /*msg*/) {
  breach_ = true;
  last_breach_time_ = std::chrono::steady_clock::now();
  write_neutral_to_all();
  RCLCPP_WARN(this->get_logger(), "BREACH detected — all thrusters neutralized");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Thruster command callback — writes PWM to Maestro
// ═══════════════════════════════════════════════════════════════════════════════

void ThrusterDriverNode::command_callback(
    const rov_msgs::msg::ThrusterCommand::SharedPtr msg) {

  // Check if breach is still active (1 second cooldown, from original)
  if (breach_) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_breach_time_);
    if (elapsed.count() > 1000) {
      breach_ = false;
    } else {
      write_neutral_to_all();
      return;
    }
  }

  for (int i = 0; i < 6; i++) {
    uint16_t target = static_cast<uint16_t>(msg->pwm[i]);
    if (msg->reverse[i]) {
      target = 3000 - target;  // Reverse direction (from original)
    }
    write_to_maestro(port_mapping_[i], target);
  }
}
