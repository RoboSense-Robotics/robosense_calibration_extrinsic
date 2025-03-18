/******************************************************************************
   Copyright 2025 RoboSense Technology Co., Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 *****************************************************************************/

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common/common_variable.h"

namespace robosense {
namespace calib {
namespace factory_calibration {
class SensorCalibration {
 private:
  std::string config_path_;
  std::string lidar_registration_config_file_;
  std::string lidar_calibration_file_;

  int cloud_size_;

  bool get_straight_done_;
  bool get_circle_done_;

  std::string straight_cloud_sub_topic_;
  std::string circle_cloud_sub_topic_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> straight_cloud_vec_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> circle_cloud_vec_;

  Eigen::Matrix4d lidar_to_sensor_tf_;
  Eigen::Matrix4d sensor_to_baselink_tf_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr straight_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr circle_subscription_;

  void loadConfigYaml(const std::string &_config_path);
  bool loadCalibrationConfig(const std::string &_config_path);
  void straightCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void circleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void startBaseCalib();

 public:
  SensorCalibration();
  ~SensorCalibration();

  void startCalib(const std::string& _config_path);
};
}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
#endif
