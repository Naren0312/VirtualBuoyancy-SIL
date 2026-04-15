// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT
//
// Ported from tools/synchronizer/src/synchronizer_bno.cpp
// Synchronizes depth + IMU data into a single SensorData message.

#include "rov_hardware/sensor_sync_node.hpp"

// ═══════════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════════

SensorSyncNode::SensorSyncNode() : Node("sensor_sync_node") {

  // ── Publisher ──────────────────────────────────────────────────────────────
  sensor_pub_ = this->create_publisher<rov_msgs::msg::SensorData>(
      "/sensor_data", 10);

  // ── Message filter subscribers ─────────────────────────────────────────────
  depth_sub_ = std::make_shared<message_filters::Subscriber<DepthMsg>>(
      this, "/depth");
  imu_sub_ = std::make_shared<message_filters::Subscriber<IMUMsg>>(
      this, "/imu_data");

  // ── Approximate time synchronizer (queue size = 50, from original) ─────────
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(50), *depth_sub_, *imu_sub_);

  sync_->registerCallback(
      std::bind(&SensorSyncNode::sync_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "SensorSyncNode initialized — waiting for /depth and /imu_data");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Synchronized callback — fuses depth + IMU into SensorData
// ═══════════════════════════════════════════════════════════════════════════════

void SensorSyncNode::sync_callback(
    const DepthMsg::ConstSharedPtr &depth_msg,
    const IMUMsg::ConstSharedPtr &imu_msg) {

  rov_msgs::msg::SensorData out;

  // Use the synchronized depth message's header stamp
  out.header.stamp = depth_msg->header.stamp;
  out.header.frame_id = "base_link";

  // Orientation: [roll, pitch, yaw] from IMU
  out.orientation[0] = imu_msg->orientation[0];
  out.orientation[1] = imu_msg->orientation[1];
  out.orientation[2] = imu_msg->orientation[2];

  // Wrap yaw to [-180, 180] (from original synchronizer_bno.cpp)
  if (out.orientation[2] > 180.0f) {
    out.orientation[2] -= 360.0f;
  }

  // Acceleration from IMU
  out.acceleration[0] = imu_msg->acceleration[0];
  out.acceleration[1] = imu_msg->acceleration[1];
  out.acceleration[2] = imu_msg->acceleration[2];

  // Depth from depth sensor
  out.depth = depth_msg->depth;

  sensor_pub_->publish(out);
}
