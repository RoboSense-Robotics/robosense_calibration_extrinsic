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

#include "calibration/robotic_arm_calibration.h"
#include "calibration/base_calibration.h"

#include "common/common_function.h"


namespace robosense {
namespace calib {
namespace factory_calibration {

RoboticArmCalibration::RoboticArmCalibration() {
  ros_node_ = std::make_shared<rclcpp::Node>("calibration_extrinsic");
}

RoboticArmCalibration::~RoboticArmCalibration() {
  rclcpp::shutdown();
  ros_node_.reset();
}

void RoboticArmCalibration::startCalib(const std::string& _config_path) {

  RCLCPP_INFO(ros_node_->get_logger(), "**** Executing Arm Calibration ... ****");

  loadConfigYaml(_config_path);

  get_pos_done_ = false;
  init_pos_stamp_ = 0;
  init_cloud_stamp_ = 0;
  cur_angle_stable_time_ = 0;
  data_stable_time_ = 0;

  cloud_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_sub_topic_, 100, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->cloudCallback(msg); });

  arm_pos_cmd_subscription_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(pos_cmd_sub_topic_, 1000, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->armPosCmdCallback(msg); });

  pos_path_publisher_ = ros_node_->create_publisher<nav_msgs::msg::Path>("pos_path", 10);
  arm_axis_publisher_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("arm_axis", 10);

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  int wait_second_thresh = 100000000;
  int wait_second = 0;
  rclcpp::Rate rate(10); // 10 hz
  RCLCPP_INFO(ros_node_->get_logger(), "Waiting For cloud ... ");
  while(rclcpp::ok() && wait_second < wait_second_thresh && get_pos_done_ == false)
  {
    rclcpp::spin_some(ros_node_);
    rate.sleep();
    wait_second++;
  }

  if (get_pos_done_ == false)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "!!!Data Wrong , Exit !!!!");
    return;
  }

  startBaseCalib();
}

void RoboticArmCalibration::loadConfigYaml(const std::string& _config_path) {
  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Config file: %f does not exist!!!", _config_path);
    return;
  }

  config_path_ = _config_path;

  YAML::Node config = YAML::LoadFile(_config_path);
  YAML::Node doc = config["LIDAR"];

  pos_angle_rad_change_min_thresh_ = DEG2RAD(10);
  pos_angle_rad_stable_thresh_ = DEG2RAD(1);

  cloud_sub_topic_ = "/rslidar_points";
  pos_cmd_sub_topic_ = "/arm_pose";
  pos_stable_time_thresh_ = 50;
  pos_ok_time_thresh_ = 3;

  if (doc["lidar_topic"])
  {
    cloud_sub_topic_ = doc["lidar_topic"].as<std::string>();
  }
  if (doc["pos_cmd_topic"])
  {
    pos_cmd_sub_topic_ = doc["pos_cmd_topic"].as<std::string>();
  }
  if (doc["pos_angle_rad_change_min_thresh"])
  {
    pos_angle_rad_change_min_thresh_ = DEG2RAD(doc["pos_angle_deg_change_min_thresh"].as<double>());
  }
  if (doc["pos_angle_rad_stable_thresh"])
  {
    pos_angle_rad_stable_thresh_ = DEG2RAD(doc["pos_angle_deg_stable_thresh"].as<double>());
  }
  if (doc["pos_stable_time_thresh"])
  {
    pos_stable_time_thresh_ = doc["pos_stable_time_thresh"].as<int>();
  }
  if (doc["pos_ok_time_thresh"])
  {
    pos_ok_time_thresh_ = doc["pos_ok_time_thresh"].as<int>();
  }
  RCLCPP_INFO(ros_node_->get_logger(), "cloud_sub_topic                     : %s ", cloud_sub_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "pos_cmd_topic                       : %s ", pos_cmd_sub_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "pos_angle_rad_change_min_thresh     : %f ", RAD2DEG(pos_angle_rad_change_min_thresh_));
  RCLCPP_INFO(ros_node_->get_logger(), "pos_angle_rad_stable_thresh         : %f ", RAD2DEG(pos_angle_rad_stable_thresh_));
  RCLCPP_INFO(ros_node_->get_logger(), "pos_stable_time_thresh              : %d ", pos_stable_time_thresh_);
  RCLCPP_INFO(ros_node_->get_logger(), "pos_ok_time_thresh                  : %d ", pos_ok_time_thresh_);

  lidar_calibration_file_ = doc["lidar_calibration_file"].as<std::string>();
  RCLCPP_INFO(ros_node_->get_logger(), "lidar_calibration_file   : %s ", lidar_calibration_file_.c_str());

  loadCalibrationConfig(lidar_calibration_file_);
}

bool RoboticArmCalibration::loadCalibrationConfig(const std::string &_config_path) {
  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "RoboticArmCalibration::loadCalibrationConfig: %s file does not exist!!!", _config_path.c_str());
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


void RoboticArmCalibration::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  if (get_pos_done_)
  {
    RCLCPP_INFO_ONCE(ros_node_->get_logger(), "**** Get PointCloud DONE !!!!!!!!!*");
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  auto stamp = msg->header.stamp;
  uint64_t sec = stamp.sec;
  uint64_t nanosec = stamp.nanosec;
  double unix_timestamp = sec + nanosec * 1e-9;

  double timestamp_key = unix_timestamp*10000;
  stamp_cloud_map_.insert(std::make_pair(timestamp_key, *cloud_ptr));
  RCLCPP_INFO(ros_node_->get_logger(), "Get PointCloud Stamp : %f , msg %f", timestamp_key, unix_timestamp);
}

void RoboticArmCalibration::armPosCmdCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (get_pos_done_)
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "**** Get Pos Cmd DONE !!!!!!!!!*");
      return;
    }

    auto stamp = msg->header.stamp;
    uint64_t sec = stamp.sec;
    uint64_t nanosec = stamp.nanosec;
    double unix_timestamp = sec + nanosec * 1e-9;
    double timestamp_key = unix_timestamp*10000;

    Eigen::Quaterniond q;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;

    std::vector<double> euler_pose = getEulerFromQuaternion(q);

    arx5roscmd cur_pos;
    cur_pos.x = msg->pose.position.x;
    cur_pos.y = msg->pose.position.y;
    cur_pos.z = msg->pose.position.z;
    cur_pos.roll = euler_pose[0];
    cur_pos.pitch = euler_pose[1];
    cur_pos.yaw = euler_pose[2];
    cur_pos.stamp = timestamp_key;

    RCLCPP_INFO(ros_node_->get_logger(), "**** cur_stamp  = %f ", unix_timestamp);
    RCLCPP_INFO(ros_node_->get_logger(), "**** x          = %f ", cur_pos.x);
    RCLCPP_INFO(ros_node_->get_logger(), "**** y          = %f ", cur_pos.y);
    RCLCPP_INFO(ros_node_->get_logger(), "**** z          = %f ", cur_pos.z);
    RCLCPP_INFO(ros_node_->get_logger(), "**** roll       = %f ", cur_pos.roll);
    RCLCPP_INFO(ros_node_->get_logger(), "**** pitch      = %f ", cur_pos.pitch);
    RCLCPP_INFO(ros_node_->get_logger(), "**** yaw        = %f ", cur_pos.yaw);

    if (stamp_pos_map_.size()==0)
    {
      init_pos_ = cur_pos;
      last_pos_ = cur_pos;
      cur_angle_stable_time_ = 0;
      data_stable_time_ = 0;
    }

    stamp_pos_map_.insert(std::make_pair(timestamp_key, cur_pos));
    RCLCPP_INFO(ros_node_->get_logger(), "**** get pos cmd !!! size = %d data_stable_time = %d", stamp_pos_map_.size(), data_stable_time_);

    pos_path_.header.stamp = ros_node_->get_clock()->now();
    pos_path_.header.frame_id = "rslidar";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = ros_node_->get_clock()->now();
    pose.header.frame_id = "rslidar";
    pose.pose.position.x = cur_pos.x;
    pose.pose.position.y = cur_pos.y;
    pose.pose.position.z = cur_pos.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pos_path_.poses.push_back(pose);

    arm_axis_publisher_->publish(pose);

    pos_path_publisher_->publish(pos_path_);

    double roll_to_last_change = std::fabs(cur_pos.roll - last_pos_.roll);
    double pitch_to_last_change = std::fabs(cur_pos.pitch - last_pos_.pitch);
    double yaw_to_last_change = std::fabs(cur_pos.yaw - last_pos_.yaw);

    if(roll_to_last_change > pos_angle_rad_stable_thresh_ || pitch_to_last_change > pos_angle_rad_stable_thresh_ || yaw_to_last_change > pos_angle_rad_stable_thresh_)
    {
      cur_angle_stable_time_ = 0;
      last_pos_ = cur_pos;
      return;
    }

    cur_angle_stable_time_++;
    RCLCPP_INFO(ros_node_->get_logger(), "**** stable time  = %d ", cur_angle_stable_time_);

    if (cur_angle_stable_time_ < pos_stable_time_thresh_)
    {
      return;
    }

    if (data_stable_time_ == 0)
    {
      stable_pos_.push_back(cur_pos);
      data_stable_time_++;
      return;
    }

    double x_to_last_stable_change = std::fabs(cur_pos.x - stable_pos_.back().x);
    double y_to_last_stable_change = std::fabs(cur_pos.y - stable_pos_.back().y);
    double z_to_last_stable_change = std::fabs(cur_pos.z - stable_pos_.back().z);
    double dis_dif = x_to_last_stable_change + y_to_last_stable_change + z_to_last_stable_change;

    double roll_to_last_stable_change = std::fabs(cur_pos.roll - stable_pos_.back().roll);
    double pitch_to_last_stable_change = std::fabs(cur_pos.pitch - stable_pos_.back().pitch);
    double yaw_to_last_stable_change = std::fabs(cur_pos.yaw - stable_pos_.back().yaw);
    double angle_dif = roll_to_last_stable_change + pitch_to_last_stable_change + yaw_to_last_stable_change;

    // RCLCPP_INFO(ros_node_->get_logger(), "**** stable time ok , roll_change   = %f ", RAD2DEG(roll_to_last_stable_change));
    // RCLCPP_INFO(ros_node_->get_logger(), "**** stable time ok , pitch_change  = %f ", RAD2DEG(pitch_to_last_stable_change));
    // RCLCPP_INFO(ros_node_->get_logger(), "**** stable time ok , yaw_change    = %f ", RAD2DEG(yaw_to_last_stable_change));

    if(dis_dif> 0.08||angle_dif > pos_angle_rad_change_min_thresh_*0.5)
    {
      stable_pos_.push_back(cur_pos);
      data_stable_time_++;
      RCLCPP_INFO(ros_node_->get_logger(), "**** !!!!! data stable time  = %d ", data_stable_time_);
    }

    if (data_stable_time_ > pos_ok_time_thresh_)
    {
      get_pos_done_ = true;
      RCLCPP_INFO(ros_node_->get_logger(), "**** get_pos_done   = True ");

      for(auto pos : stable_pos_)
      {
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose stamp %f = ", pos.stamp);
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose x     %f = ", pos.x);
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose y     %f = ", pos.y);
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose z     %f = ", pos.z);
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose roll  %f = ", RAD2DEG(pos.roll));
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose pitch %f = ", RAD2DEG(pos.pitch));
        RCLCPP_INFO(ros_node_->get_logger(), "**** get_stable_pose yaw   %f = ", RAD2DEG(pos.yaw));
      }
    }
}

void RoboticArmCalibration::insertSelectTimeData(const double & select_stamp)
{
  RCLCPP_INFO(ros_node_->get_logger(), "get select_pos  select_stamp = %f ", select_stamp);

  bool get_pos = false;
  arx5roscmd select_pos;

  double min_dif = 100000000000;

  for (auto pos_info : stamp_pos_map_)
  {
    double cur_dif = std::fabs(select_stamp - pos_info.first);
    if (cur_dif < min_dif)
    {
      select_pos = pos_info.second;
      min_dif = cur_dif;
    }
    else
    {
      get_pos = true;
      RCLCPP_INFO(ros_node_->get_logger(), "select_pos.stamp  = %f ", select_pos.stamp);
      break;
    }
  }

  if(!get_pos)
  {
    return;
  }

  min_dif = 100000000000;

  pcl::PointCloud<pcl::PointXYZI> select_cloud;
  for (auto cloud_info : stamp_cloud_map_)
  {
    double cur_dif = std::fabs(select_stamp - cloud_info.first);
    if (cur_dif < min_dif)
    {
      select_cloud = cloud_info.second;
      min_dif = cur_dif;
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "cloud_info.stamp  = %f ", cloud_info.first);
      break;
    }
  }

  if(select_cloud.size()==0)
  {
    return;
  }

  std::vector<double> cur_pose = {select_pos.roll, select_pos.pitch, select_pos.yaw, select_pos.x, select_pos.y, select_pos.z};

  Eigen::Matrix4d pos_tf = getMatrixFromEulerPose(cur_pose);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_select_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  double range_thresh_2 = 2*2;
  for(auto p:select_cloud)
  {
    if(pcl::isFinite(p)==false)
    {
      continue;
    }
    double cur_dis = p.x*p.x + p.y*p.y + p.z*p.z;

    if (cur_dis > range_thresh_2)
    {
      filter_select_cloud->points.push_back(p);
    }

  }

  select_cloud_vec_.emplace_back(*filter_select_cloud);
  select_pos_tf_vec_.emplace_back(pos_tf.cast<float>());

  RCLCPP_INFO(ros_node_->get_logger(), "get data x      = %f ", select_pos.x);
  RCLCPP_INFO(ros_node_->get_logger(), "get data y      = %f ", select_pos.y);
  RCLCPP_INFO(ros_node_->get_logger(), "get data z      = %f ", select_pos.z);
  RCLCPP_INFO(ros_node_->get_logger(), "get data roll   = %f ", RAD2DEG(select_pos.roll));
  RCLCPP_INFO(ros_node_->get_logger(), "get data pitch  = %f ", RAD2DEG(select_pos.pitch));
  RCLCPP_INFO(ros_node_->get_logger(), "get data yaw    = %f ", RAD2DEG(select_pos.yaw));
  RCLCPP_INFO(ros_node_->get_logger(), "---------------- get data size   = %d ", select_pos_tf_vec_.size());
}

void RoboticArmCalibration::getFilterData()
{
  int stable_time = 0;

  arx5roscmd last_pos = init_pos_;
  arx5roscmd cur_pos = init_pos_;

  double trans_diff_thresh = 0.1;
  double angle_rad_diff_thresh = DEG2RAD(1);

  double start_stamp = 0;
  double end_stamp = 0;

  int count = 0;

  for (auto pos_info : stamp_pos_map_)
  {
    cur_pos = pos_info.second;
    double x_trans_dif = std::fabs(pos_info.second.x - last_pos.x);
    double y_trans_dif = std::fabs(pos_info.second.y - last_pos.y);
    double z_trans_dif = std::fabs(pos_info.second.z - last_pos.z);
    double roll_dif    = std::fabs(pos_info.second.roll - last_pos.roll);
    double pitch_dif   = std::fabs(pos_info.second.pitch - last_pos.pitch);
    double yaw_dif     = std::fabs(pos_info.second.yaw - last_pos.yaw);

    double trans_dif = x_trans_dif + y_trans_dif + z_trans_dif;
    double angle_dif = roll_dif + pitch_dif + yaw_dif;

    count++;
    if (trans_dif < trans_diff_thresh && angle_dif < angle_rad_diff_thresh)
    {
      stable_time++;
      end_stamp = pos_info.first;
      continue;
    }

    if (stable_time > pos_stable_time_thresh_)
    {
      double mid_stamp = (end_stamp + start_stamp) * 0.5;
      insertSelectTimeData(mid_stamp);
    }

    stable_time = 0;
    last_pos = pos_info.second;
    start_stamp = pos_info.first;
  }

  if (stable_time > pos_stable_time_thresh_)
  {
    double mid_stamp = (cur_pos.stamp + start_stamp) * 0.5;
    insertSelectTimeData(mid_stamp);
  }
}

void RoboticArmCalibration::startBaseCalib() {
  clock_t start_time = clock();

  RCLCPP_INFO(ros_node_->get_logger(), "========== start Calibration ==========");

  getFilterData();

  BaseCalibration base_calib;
  if (base_calib.loadConfig(config_path_, ros_node_)) {
    // calibration lidar_to_arm
    std::vector<double> lidar_to_arm_pose = base_calib.executeLidarToArmCalibration(select_cloud_vec_, select_pos_tf_vec_);

    auto quaterion = getQuaterionFromEulerPose(lidar_to_arm_pose);
    RCLCPP_INFO(ros_node_->get_logger(), "========== Lidar To Arm Trans : x    = %f , y     = %f , z   = %f ", lidar_to_arm_pose[3], lidar_to_arm_pose[4], lidar_to_arm_pose[5]);
    RCLCPP_INFO(ros_node_->get_logger(), "========== Lidar To Arm Eular : roll = %f , pitch = %f , yaw = %f ", RAD2DEG(lidar_to_arm_pose[0]), RAD2DEG(lidar_to_arm_pose[1]), RAD2DEG(lidar_to_arm_pose[2]));
    RCLCPP_INFO(ros_node_->get_logger(), "========== Lidar To Arm Quaterion(w, x, y, z) : %f , %f , %f , %f", quaterion.w(), quaterion.x(), quaterion.y(), quaterion.z());
  }
  clock_t end_time = clock();
  double time_duration =
      static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;

  RCLCPP_DEBUG(ros_node_->get_logger(), "========== Calibration finished, time cost %f s ==========", time_duration);
}

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
