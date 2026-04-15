// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rov_msgs/msg/thruster_command.hpp"

#include <array>
#include <chrono>
#include <string>

class ThrusterDriverNode : public rclcpp::Node {
public:
  ThrusterDriverNode();
  ~ThrusterDriverNode() override;

private:
  void command_callback(const rov_msgs::msg::ThrusterCommand::SharedPtr msg);
  void breach_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void write_neutral_to_all();
  void write_to_maestro(int port_num, uint16_t target);

  rclcpp::Subscription<rov_msgs::msg::ThrusterCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr breach_sub_;

  std::array<int, 6> port_mapping_{};
  std::string serial_port_;
  int fd_{-1};  // serial file descriptor

  bool breach_{false};
  std::chrono::steady_clock::time_point last_breach_time_;
};
