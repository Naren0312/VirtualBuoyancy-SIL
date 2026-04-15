// Copyright (c) 2017 Tiburon, NIT Rourkela — ROS2 port
// SPDX-License-Identifier: MIT

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rov_msgs/msg/depth.hpp"
#include "rov_msgs/msg/imu_data.hpp"
#include "rov_msgs/msg/sensor_data.hpp"

class SensorSyncNode : public rclcpp::Node {
public:
  SensorSyncNode();
  ~SensorSyncNode() override = default;

private:
  using DepthMsg = rov_msgs::msg::Depth;
  using IMUMsg   = rov_msgs::msg::IMUData;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<DepthMsg, IMUMsg>;

  void sync_callback(const DepthMsg::ConstSharedPtr &depth_msg,
                     const IMUMsg::ConstSharedPtr &imu_msg);

  rclcpp::Publisher<rov_msgs::msg::SensorData>::SharedPtr sensor_pub_;

  std::shared_ptr<message_filters::Subscriber<DepthMsg>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<IMUMsg>>   imu_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
