// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Ported from hardware_stack/hammerhead_serial/scripts/serial_node.py
// Rewritten in C++ for consistency and performance.
// Reads Arduino serial data, parses the custom protocol (D, Y, P, R keys),
// publishes /depth and /imu_data topics.

#include "rov_hardware/serial_bridge_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <map>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════════

SerialBridgeNode::SerialBridgeNode() : Node("serial_bridge_node") {

  // ── Parameters ─────────────────────────────────────────────────────────────
  this->declare_parameter("serial_port", "/dev/nuc_nano");
  this->declare_parameter("baud_rate", 9600);

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_   = static_cast<int>(this->get_parameter("baud_rate").as_int());

  // ── Open serial port ──────────────────────────────────────────────────────
  fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ == -1) {
    RCLCPP_ERROR(this->get_logger(), "Could not open serial device: %s",
                 serial_port_.c_str());
    // Do not throw; gracefully wait. The read timer will just early exit if fd_ == -1.
    return;
  }

  // Configure serial port
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  tcgetattr(fd_, &tty);

  // Set baud rate
  speed_t baud;
  switch (baud_rate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    default:     baud = B9600;   break;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  // 100ms timeout

  tcsetattr(fd_, TCSANOW, &tty);

  RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud",
              serial_port_.c_str(), baud_rate_);

  // ── Publishers ─────────────────────────────────────────────────────────────
  depth_pub_ = this->create_publisher<rov_msgs::msg::Depth>("/depth", 10);
  imu_pub_   = this->create_publisher<rov_msgs::msg::IMUData>("/imu_data", 10);

  // ── Subscriber (status forwarding to Arduino) ──────────────────────────────
  status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/thruster_status", 10,
      std::bind(&SerialBridgeNode::status_callback, this,
                std::placeholders::_1));

  // ── Timer for reading serial at ~30 Hz ─────────────────────────────────────
  read_timer_ = this->create_wall_timer(
      33ms, std::bind(&SerialBridgeNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "SerialBridgeNode initialized");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Destructor
// ═══════════════════════════════════════════════════════════════════════════════

SerialBridgeNode::~SerialBridgeNode() {
  if (fd_ != -1) {
    close(fd_);
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Timer callback — reads serial data and parses complete lines
// ═══════════════════════════════════════════════════════════════════════════════

void SerialBridgeNode::timer_callback() {
  if (fd_ == -1) return;

  char buf[256];
  ssize_t n = read(fd_, buf, sizeof(buf) - 1);

  if (n <= 0) return;

  buf[n] = '\0';

  // Accumulate into line buffer, process complete lines
  for (ssize_t i = 0; i < n; i++) {
    if (buf[i] == '\n' || buf[i] == '\r') {
      if (!line_buffer_.empty()) {
        parse_serial_line(line_buffer_);
        line_buffer_.clear();
      }
    } else {
      line_buffer_ += buf[i];
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Parse a serial line using the original protocol:
//   D<value>#Y<value>#P<value>#R<value>#
// Keys: D=depth, Y=yaw, P=pitch, R=roll
// ═══════════════════════════════════════════════════════════════════════════════

void SerialBridgeNode::parse_serial_line(const std::string &line) {

  // Protocol key → index mapping (from original serial_node.py)
  // D=0 (depth), Y=9 (yaw), P=10 (pitch), R=11 (roll)
  static const std::map<char, int> key_map = {
      {'D', 0}, {'Y', 9}, {'P', 10}, {'R', 11}};

  float values[13];
  for (int i = 0; i < 13; i++) values[i] = -1.0f;

  char current_key = '\0';
  std::string current_value;
  bool reading_value = false;

  for (size_t i = 0; i < line.size(); i++) {
    char c = line[i];

    if (std::isalpha(c)) {
      current_key = c;
      current_value.clear();
      reading_value = true;
    } else if (reading_value && (std::isdigit(c) || c == '-' || c == '.')) {
      current_value += c;
    } else if (c == '#') {
      if (current_key != '\0' && !current_value.empty()) {
        auto it = key_map.find(current_key);
        if (it != key_map.end()) {
          try {
            values[it->second] = std::stof(current_value);
          } catch (...) {
            // malformed value, skip
          }
        }
      }
      current_key = '\0';
      current_value.clear();
      reading_value = false;
    }
  }

  // ── Publish depth ──────────────────────────────────────────────────────────
  if (values[0] != -1.0f) {
    rov_msgs::msg::Depth depth_msg;
    depth_msg.header.stamp = this->now();
    depth_msg.header.frame_id = "depth_sensor";
    depth_msg.depth = values[0];
    depth_pub_->publish(depth_msg);
  }

  // ── Publish IMU data (roll, pitch, yaw) ────────────────────────────────────
  if (values[9] != -1.0f) {
    rov_msgs::msg::IMUData imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_sensor";
    // orientation order: [roll, pitch, yaw]
    // Original serial_node.py: bno.orientation = [R, P, Y]
    imu_msg.orientation[0] = values[11];  // Roll
    imu_msg.orientation[1] = values[10];  // Pitch
    imu_msg.orientation[2] = values[9];   // Yaw
    // No acceleration data from BNO in original protocol
    imu_msg.acceleration[0] = 0.0f;
    imu_msg.acceleration[1] = 0.0f;
    imu_msg.acceleration[2] = 0.0f;
    imu_pub_->publish(imu_msg);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Status callback — forwards thruster status to Arduino via serial
// ═══════════════════════════════════════════════════════════════════════════════

void SerialBridgeNode::status_callback(
    const std_msgs::msg::Int8::SharedPtr msg) {
  if (fd_ == -1) return;

  // Protocol: S<value>#  (from original serial_node.py statusCallback)
  std::string cmd = "S" + std::to_string(msg->data) + "#";
  ssize_t written = write(fd_, cmd.c_str(), cmd.size());
  if (written == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error writing status to serial");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Sent status: %s", cmd.c_str());
  }
}
