// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include "rov_msgs/msg/depth.hpp"
#include "rov_msgs/msg/imu_data.hpp"

#include <string>

class SerialBridgeNode : public rclcpp::Node {
public:
  SerialBridgeNode();
  ~SerialBridgeNode() override;

private:
  void timer_callback();
  void status_callback(const std_msgs::msg::Int8::SharedPtr msg);
  void parse_serial_line(const std::string &line);

  rclcpp::Publisher<rov_msgs::msg::Depth>::SharedPtr depth_pub_;
  rclcpp::Publisher<rov_msgs::msg::IMUData>::SharedPtr imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  std::string serial_port_;
  int baud_rate_;
  int fd_{-1};

  // Buffer for partial line reads
  std::string line_buffer_;
};
