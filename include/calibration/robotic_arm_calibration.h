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

#ifndef ROBOTIC_ARM_CALIBRATION_H
#define ROBOTIC_ARM_CALIBRATION_H

#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "common/common_variable.h"
#include "common/common_function.h"

namespace robosense {
namespace calib {
namespace factory_calibration {

struct arx5roscmd
{
  float stamp = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};

class RoboticArmCalibration {
 private:
  std::string config_path_;
  std::string lidar_calibration_file_;

  std::string cloud_sub_topic_;
  std::string pos_cmd_sub_topic_;

  double pos_angle_rad_change_min_thresh_;
  double pos_angle_rad_stable_thresh_;
  int pos_stable_time_thresh_;
  int pos_ok_time_thresh_;
  int cur_angle_stable_time_;
  int data_stable_time_;

  bool get_pos_done_;

  arx5roscmd init_pos_;
  arx5roscmd last_pos_;
  std::vector<arx5roscmd> stable_pos_;

  double init_cloud_stamp_;
  double init_pos_stamp_;

  std::map<double, pcl::PointCloud<pcl::PointXYZI>> stamp_cloud_map_;
  std::map<double, arx5roscmd> stamp_pos_map_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> select_cloud_vec_;
  std::vector<Eigen::Matrix4f> select_pos_tf_vec_;
  
  Eigen::Matrix4d lidar_to_sensor_tf_;
  Eigen::Matrix4d sensor_to_baselink_tf_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pos_cmd_subscription_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pos_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_axis_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path pos_path_;

  void loadConfigYaml(const std::string &_config_path);
  bool loadCalibrationConfig(const std::string &_config_path);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void armPosCmdCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void getFilterData();
  void insertSelectTimeData(const double &select_stamp);
  void startBaseCalib();

public:
  RoboticArmCalibration();
  ~RoboticArmCalibration();

  void startCalib(const std::string& _config_path);

  void handEyeCali(const std::string _pose_path);
};
}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
#endif
