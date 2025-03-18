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

#include <unistd.h>

#include <iostream>

#include <eigen3/Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "calibration/sensor_calibration.h"
#include "calibration/base_calibration.h"
#include "common/common_function.h"


namespace robosense {
namespace calib {
namespace factory_calibration {

SensorCalibration::SensorCalibration() {
  ros_node_ = std::make_shared<rclcpp::Node>("robosense_calibration_extrinsic");
}

SensorCalibration::~SensorCalibration() {
  rclcpp::shutdown();
  ros_node_.reset();
}

void SensorCalibration::startCalib(const std::string& _config_path) {

  RCLCPP_INFO(ros_node_->get_logger(), "**** Executing Calibration ... ****");

  loadConfigYaml(_config_path);

  get_straight_done_ = false;
  get_circle_done_ = false;

  straight_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(straight_cloud_sub_topic_, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->straightCloudCallback(msg); });
  circle_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(circle_cloud_sub_topic_, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->circleCloudCallback(msg); });

  int wait_second_thresh = 10000000;
  int wait_second = 0;
  rclcpp::Rate rate(10); // 10 hz
  RCLCPP_INFO(ros_node_->get_logger(), "Waiting For cloud ... ");
  while(rclcpp::ok() && wait_second < wait_second_thresh && (get_straight_done_ == false || get_circle_done_ == false))
  {
    rclcpp::spin_some(ros_node_);
    rate.sleep();
    wait_second++;
  }

  if (get_straight_done_ == false || get_circle_done_ == false)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "!!!Data Wrong , Exit !!!!");
    return;
  }

  startBaseCalib();
}

void SensorCalibration::loadConfigYaml(const std::string& _config_path) {
  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Config file: %f does not exist!!!", _config_path);
    return;
  }

  config_path_ = _config_path;

  YAML::Node config = YAML::LoadFile(_config_path);
  YAML::Node doc = config["LIDAR"];

  cloud_size_ = 10;
  straight_cloud_sub_topic_ = "/straight/rslidar_points_e1";
  circle_cloud_sub_topic_ = "/circle/rslidar_points_e1";

  if (doc["cloud_size"])
  {
    cloud_size_ = doc["cloud_size"].as<int>();
  }
  if (doc["straight_lidar_topic"])
  {
    straight_cloud_sub_topic_ = doc["straight_lidar_topic"].as<std::string>();
  }
  if (doc["circle_lidar_topic"])
  {
    circle_cloud_sub_topic_ = doc["circle_lidar_topic"].as<std::string>();
  }
  RCLCPP_INFO(ros_node_->get_logger(), "straight_cloud_sub_topic : %s ", straight_cloud_sub_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "circle_cloud_sub_topic   : %s ", circle_cloud_sub_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "cloud_size               : %d ", cloud_size_);


  lidar_calibration_file_ = doc["lidar_calibration_file"].as<std::string>();
  RCLCPP_INFO(ros_node_->get_logger(), "lidar_calibration_file   : %s ", lidar_calibration_file_.c_str());

  loadCalibrationConfig(lidar_calibration_file_);
}

bool SensorCalibration::loadCalibrationConfig(const std::string &_config_path) {
  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "SensorCalibration::loadCalibrationConfig: %s file does not exist!!!", _config_path.c_str());
    return false;
  }
  YAML::Node config = YAML::LoadFile(_config_path);
  YAML::Node sensor_to_baselink = config["Body"]["extrinsic"];
  YAML::Node lidar_to_sensor = config["Sensor"]["Lidar"]["extrinsic"];

  std::vector<double> sensor_to_baselink_pose;
  std::vector<double> lidar_to_sensor_pose;

  Eigen::Quaterniond sensor_to_baselink_q;
  Eigen::Quaterniond lidar_to_sensor_q;

  sensor_to_baselink_pose.push_back(sensor_to_baselink["translation"]["x"].as<double>());
  sensor_to_baselink_pose.push_back(sensor_to_baselink["translation"]["y"].as<double>());
  sensor_to_baselink_pose.push_back(sensor_to_baselink["translation"]["z"].as<double>());

  sensor_to_baselink_q.x() = sensor_to_baselink["quaternion"]["x"].as<double>();
  sensor_to_baselink_q.y() = sensor_to_baselink["quaternion"]["y"].as<double>();
  sensor_to_baselink_q.z() = sensor_to_baselink["quaternion"]["z"].as<double>();
  sensor_to_baselink_q.w() = sensor_to_baselink["quaternion"]["w"].as<double>();

  lidar_to_sensor_pose.push_back(lidar_to_sensor["translation"]["x"].as<double>());
  lidar_to_sensor_pose.push_back(lidar_to_sensor["translation"]["y"].as<double>());
  lidar_to_sensor_pose.push_back(lidar_to_sensor["translation"]["z"].as<double>());

  lidar_to_sensor_q.x() = lidar_to_sensor["quaternion"]["x"].as<double>();
  lidar_to_sensor_q.y() = lidar_to_sensor["quaternion"]["y"].as<double>();
  lidar_to_sensor_q.z() = lidar_to_sensor["quaternion"]["z"].as<double>();
  lidar_to_sensor_q.w() = lidar_to_sensor["quaternion"]["w"].as<double>();

  sensor_to_baselink_tf_ = getMatrixFromTransAndQuaterion(sensor_to_baselink_pose, sensor_to_baselink_q);
  lidar_to_sensor_tf_ = getMatrixFromTransAndQuaterion(lidar_to_sensor_pose, lidar_to_sensor_q);

  return true;
}


void SensorCalibration::straightCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  if (straight_cloud_vec_.size() > size_t(cloud_size_))
  {
    get_straight_done_ = true;
    RCLCPP_INFO_ONCE(ros_node_->get_logger(), "**** Get Straight PointCloud DONE !!!!!!!!!*");
    return;
  }
  // 将ROS消息转换为PCL点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);
  straight_cloud_vec_.emplace_back(*cloud_ptr);
  RCLCPP_INFO(ros_node_->get_logger(), "Get Straight PointCloud Size : %d ", straight_cloud_vec_.size());
}

void SensorCalibration::circleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  if (circle_cloud_vec_.size() > size_t(cloud_size_))
  {
    get_circle_done_ = true;
    RCLCPP_INFO_ONCE(ros_node_->get_logger(), "**** Get Circle PointCloud DONE !!!!!!!!!*");
    return;
  }
  // 将ROS消息转换为PCL点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);
  circle_cloud_vec_.emplace_back(*cloud_ptr);

  RCLCPP_INFO(ros_node_->get_logger(), "Get Circle PointCloud Size : %d ", circle_cloud_vec_.size());
}

void SensorCalibration::startBaseCalib() {
  clock_t start_time = clock();

  RCLCPP_INFO(ros_node_->get_logger(), "========== start Calibration ==========");

  BaseCalibration base_calib;
  if (base_calib.loadConfig(config_path_, ros_node_)) {
    // calibration lidar_to_baselink
    auto lidar_to_baselink_eular_pose = base_calib.executeCalibration(straight_cloud_vec_, circle_cloud_vec_);
    auto quaterion = getQuaterionFromEulerPose(lidar_to_baselink_eular_pose);
    RCLCPP_INFO(ros_node_->get_logger(), "========== Lidar To Baselink Eular : roll = %f , pitch = %f , yaw = %f ", RAD2DEG(lidar_to_baselink_eular_pose[0]), RAD2DEG(lidar_to_baselink_eular_pose[1]), RAD2DEG(lidar_to_baselink_eular_pose[2]));
    RCLCPP_INFO(ros_node_->get_logger(), "========== Lidar To Baselink Quaterion(w, x, y, z) : %f , %f , %f , %f", quaterion.w(), quaterion.x(), quaterion.y(), quaterion.z());
    // get sensor_to_baselink according lidar_to_baselink and lidar_to_sensor

    Eigen::Matrix4d lidar_to_baselink_tf = getMatrixFromEulerPose(lidar_to_baselink_eular_pose);

    Eigen::Matrix4d sensor_to_baselink_tf = lidar_to_sensor_tf_.inverse() * lidar_to_baselink_tf;

    std::vector<double> sensor_to_baselink_euler_pose = getEulerPoseFromMatrix(sensor_to_baselink_tf);
    Eigen::Quaterniond sensor_to_baselink_q = getQuaterionFromEulerPose(sensor_to_baselink_euler_pose);

    RCLCPP_INFO(ros_node_->get_logger(), "========== Sensor To Baselink Quaterion(w, x, y, z) : %f , %f , %f , %f", sensor_to_baselink_q.w(), sensor_to_baselink_q.x(), sensor_to_baselink_q.y(), sensor_to_baselink_q.z());
  }
  clock_t end_time = clock();
  double time_duration =
      static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;

  RCLCPP_DEBUG(ros_node_->get_logger(), "========== Calibration finished, time cost %f s ==========", time_duration);
}

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
