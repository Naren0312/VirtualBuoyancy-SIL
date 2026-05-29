// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#include "rov_control/control_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  // Use MultiThreadedExecutor so mode_callback and setpoint_callback
  // are never starved by the high-rate (30 Hz) sensor_data_callback.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
