// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#include "rov_hardware/sensor_sync_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSyncNode>());
  rclcpp::shutdown();
  return 0;
}
