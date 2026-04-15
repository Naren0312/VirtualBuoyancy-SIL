// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#include "rov_hardware/serial_bridge_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
