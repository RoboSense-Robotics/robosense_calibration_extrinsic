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

#include "calibration/base_calibration.h"
#include "common/common_function.h"

#include <yaml-cpp/yaml.h>

namespace robosense {
namespace calib {
namespace factory_calibration {
BaseCalibration::BaseCalibration() {}

BaseCalibration::~BaseCalibration() {}

bool BaseCalibration::loadConfig(const std::string _config_path, rclcpp::Node::SharedPtr _ros_node) {
  ros_node_ = _ros_node;

  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "BaseCalibration::loadConfig: %s file does not exist!!!", _config_path.c_str());
    return false;
  }

  YAML::Node config = YAML::LoadFile(_config_path);
  YAML::Node doc = config["BASE"];

  anticlockwise_ = false;
  forward_ = false;
  save_path_ = "";

  if (doc["anticlockwise"])
  {
    anticlockwise_ = doc["anticlockwise"].as<bool>();
  }
  if (doc["forward"])
  {
    forward_ = doc["forward"].as<bool>();
  }
  if (doc["save_path"])
  {
    save_path_ = doc["save_path"].as<std::string>();
  }
  registration_config_path_ = doc["lidar_registration_config_file"].as<std::string>();

  if (!loadRegistrationConfig(registration_config_path_))
  {
    return false;
  }

  registrationParamSet();

  return true;
}

bool BaseCalibration::loadRegistrationConfig(const std::string _config_path) {
  if (access(_config_path.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(ros_node_->get_logger(), "BaseCalibration::loadRegistrationConfig: %s file does not exist!!!", _config_path.c_str());
    return false;
  }
  YAML::Node config = YAML::LoadFile(_config_path);
  YAML::Node ndt_config = config["NDT"];
  ndt_param_.leaf_size = ndt_config["LeafSize"].as<double>();
  ndt_param_.step_size = ndt_config["StepSize"].as<double>();
  ndt_param_.resolution = ndt_config["Resolution"].as<double>();
  ndt_param_.max_iteration = ndt_config["MaxIteration"].as<int>();
  ndt_param_.epsilon = ndt_config["Epsilon"].as<double>();

  return true;
}

void BaseCalibration::registrationParamSet()
{
  // -----------------设置ndt相关参数-----------------------
  ndt_.setStepSize(ndt_param_.step_size);
  ndt_.setResolution(ndt_param_.resolution);
  ndt_.setMaximumIterations(ndt_param_.max_iteration);
  ndt_.setTransformationEpsilon(ndt_param_.epsilon);
}

std::vector<double> BaseCalibration::executeLidarToBodyCalibration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _straight_cloud_vec,
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & _circle_cloud_vec) {

  RCLCPP_INFO(ros_node_->get_logger(), "********** Step 1, registration **********");
  calculating_type = "straight";
  Eigen::Matrix4d straight_ndt = getPointCloudFirstToLastRegistration(_straight_cloud_vec);
  calculating_type = "circle";
  Eigen::Matrix4d circle_ndt = getPointCloudFirstToLastRegistration(_circle_cloud_vec);
  RCLCPP_INFO(ros_node_->get_logger(), "********** Step 2, calculate circleation **********");
  return calRotationResult(straight_ndt, circle_ndt, anticlockwise_, forward_);
}

std::vector<double> BaseCalibration::executeLidarToArmCalibration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _cloud_vec, 
    std::vector<Eigen::Matrix4f> & _pos_tf_vec){

  RCLCPP_INFO(ros_node_->get_logger(), "********** Step 1, registration **********");

  std::vector<Eigen::Matrix4f> cloud_match_tf = getPointCloudToFirstRegistration(_cloud_vec, _pos_tf_vec);

  RCLCPP_INFO(ros_node_->get_logger(), "********** Step 2, calculate pose **********");

  std::vector<Eigen::Matrix4d> select_cloud_tf;
  std::vector<Eigen::Matrix4d> select_pos_tf;

  for(size_t i = 0; i < cloud_match_tf.size();i++)
  {
    Eigen::Matrix4f cur_to_first_arm_pos_tf = _pos_tf_vec[0].inverse()*_pos_tf_vec[i];
    select_cloud_tf.push_back(cloud_match_tf[i].cast<double>());
    select_pos_tf.push_back(cur_to_first_arm_pos_tf.cast<double>());
  }

  std::vector<double> lidar_to_arm_pose = {0,0,0,0,0,0};

  if (select_cloud_tf.size() < 2)
  {
    return lidar_to_arm_pose;
  }

  Eigen::Matrix4d cali_tf;
  if(handEyeCalibrationNavy(select_pos_tf, select_cloud_tf, cali_tf))
  {
    lidar_to_arm_pose = getEulerPoseFromMatrix(cali_tf);
  }

  return lidar_to_arm_pose;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BaseCalibration::getNDTMatch(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& _source_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& _target_cloud,
    const Eigen::Matrix4f& _init_tf, Eigen::Matrix4f& _match_tf) {

  _match_tf.setIdentity();

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_.setInputSource(_source_cloud);
  ndt_.setInputTarget(_target_cloud);
  ndt_.align(*transformed_cloud, _init_tf);

  if (ndt_.hasConverged()) {
    _match_tf = ndt_.getFinalTransformation();
    auto score = ndt_.getFitnessScore();
    int iteration_num = ndt_.getFinalNumIteration();
    RCLCPP_INFO(ros_node_->get_logger(), "NDT Done - iteration num: %d - score: %f ", iteration_num, score);

    auto init_pose = getEulerPoseFromMatrix(_init_tf.cast<double>());
    std::string init_pose_str = "init  eular pose: ";
    for (auto it : init_pose) {
      init_pose_str += (std::to_string(it) + ", ");
    }

    auto pose = getEulerPoseFromMatrix(_match_tf.cast<double>());
    std::string pose_str = "match eular pose: ";
    for (auto it : pose) {
      pose_str += (std::to_string(it) + ", ");
    }
    RCLCPP_INFO(ros_node_->get_logger(), "NDT %s ", init_pose_str.c_str());
    RCLCPP_INFO(ros_node_->get_logger(), "NDT %s ", pose_str.c_str());
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "NDT did not converge!!!");
  }

  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BaseCalibration::getFilterCloud(pcl::PointCloud<pcl::PointXYZI> & input_cloud)
{
  // 设置点云为非密集（包含NaN点）
    input_cloud.is_dense = false;
    // 创建一个输出点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 移除NaN点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(input_cloud, *output_cloud, indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    octreeFilter(output_cloud, filter_cloud, ndt_param_.leaf_size);
    return filter_cloud;
}

Eigen::Matrix4d BaseCalibration::getPointCloudFirstToLastRegistration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _straight_cloud_vec)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Start Registrating %s Frame ... ", calculating_type.c_str());

  Eigen::Matrix4f init_tf = Eigen::Matrix4f::Identity();

  if (_straight_cloud_vec.size()< 2)
  {
    return init_tf.cast<double>();
  }

  Eigen::Matrix4f cur_to_start_tf = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZI>::Ptr last_cloud = getFilterCloud(_straight_cloud_vec[0]);

  std::string cur_path = "";
  if (save_path_.length()>0)
  {
    cur_path = save_path_ + "/" + calculating_type + "_first_frame.pcd";
    pcl::io::savePCDFile(cur_path, *last_cloud);
  }

  std::deque<pcl::PointCloud<pcl::PointXYZI>> history_map_vec;
  history_map_vec.push_back(*last_cloud);

  size_t map_size = 10;

  for (size_t i = 1; i<_straight_cloud_vec.size();i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud = getFilterCloud(_straight_cloud_vec[i]);
    Eigen::Matrix4f cur_to_first_tf = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_last_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    octreeFilter(last_cloud, filter_last_cloud, ndt_param_.leaf_size);

    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cur_cloud = getNDTMatch(cur_cloud, filter_last_cloud, cur_to_start_tf, cur_to_first_tf);
    history_map_vec.push_back(*trans_cur_cloud);

    if (history_map_vec.size() > map_size)
    {
      history_map_vec.pop_front();
    }

    last_cloud->clear();
    for(auto history_cloud: history_map_vec)
    {
      *last_cloud += history_cloud;
    }

    cur_to_start_tf = cur_to_first_tf;

    RCLCPP_INFO(ros_node_->get_logger(), "Registrating Iter %d , cur_cloud Size %d ", i, cur_cloud->size());

    if (save_path_.length()>0)
    {
      cur_path = save_path_ + "/" + calculating_type + "_iter_"+ std::to_string(i) + "_cur_to_first_frame.pcd";
      pcl::io::savePCDFile(cur_path, *trans_cur_cloud);
    }
  }

  return cur_to_start_tf.cast<double>();
}

std::vector<Eigen::Matrix4f> BaseCalibration::getPointCloudToFirstRegistration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _cloud_vec, std::vector<Eigen::Matrix4f> & _pos_tf_vec)
{
  std::vector<Eigen::Matrix4f> cloud_tf_vec;

  if (_cloud_vec.size()!=_pos_tf_vec.size() || _cloud_vec.size() < 3)
  {
    return cloud_tf_vec;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr first_cloud(new pcl::PointCloud<pcl::PointXYZI>(_cloud_vec[0]));

  Eigen::Matrix4f first_tf = Eigen::Matrix4f::Identity();
  cloud_tf_vec.push_back(first_tf);


  std::string cur_path = "";
  if (save_path_.length()>0)
  {
    cur_path = save_path_ + "/cali_to_arm_first_frame.pcd";
    first_cloud->height = 1;
    first_cloud->width = first_cloud->size();
    pcl::io::savePCDFile(cur_path, *first_cloud);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_last_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  octreeFilter(first_cloud, filter_last_cloud, ndt_param_.leaf_size);

  pcl::PointCloud<pcl::PointXYZI>::Ptr first_map(new pcl::PointCloud<pcl::PointXYZI>(_cloud_vec[0]));

  for(size_t i = 1; i < _cloud_vec.size();i++)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "--------- Registrating - iter  %d", i);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>(_cloud_vec[i]));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cur_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    octreeFilter(cur_cloud, filter_cur_cloud, ndt_param_.leaf_size);

    Eigen::Matrix4f last_tf = _pos_tf_vec[i-1];
    Eigen::Matrix4f cur_tf = _pos_tf_vec[i];
    Eigen::Matrix4f cur_to_last_tf = last_tf.inverse()*cur_tf;
    Eigen::Matrix4f cur_to_first_init_tf = cloud_tf_vec.back()*(cur_to_last_tf);
    Eigen::Matrix4f cur_to_first_match_tf = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_first_map(new pcl::PointCloud<pcl::PointXYZI>());
    octreeFilter(first_map, filter_first_map, ndt_param_.leaf_size);
    getNDTMatch(filter_cur_cloud, filter_first_map, cur_to_first_init_tf, cur_to_first_match_tf);

    cloud_tf_vec.push_back(cur_to_first_match_tf);

    if (save_path_.length()>0)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cur_to_first_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*cur_cloud, *trans_cur_to_first_cloud, cur_to_first_match_tf);
      cur_path = save_path_ + "/cali_to_arm_iter_"+ std::to_string(i) + "_cur_to_first.pcd";
      trans_cur_to_first_cloud->height = 1;
      trans_cur_to_first_cloud->width = trans_cur_to_first_cloud->size();
      pcl::io::savePCDFile(cur_path, *trans_cur_to_first_cloud);

      *first_map += *trans_cur_to_first_cloud;
    }

    filter_last_cloud->clear();
    *filter_last_cloud = *filter_cur_cloud;
  }

  if (save_path_.length()>0)
  {
    std::string save_pos_path = save_path_ + "/arm_pose.txt";
    std::string save_cloud_pose_path = save_path_ + "/cloud_pose.txt";
    std::ofstream pos_file;
    std::ofstream cloud_file;
    pos_file.open(save_pos_path);
    cloud_file.open(save_cloud_pose_path);

    for (std::size_t i = 0; i < cloud_tf_vec.size();i++)
    {
      std::vector<double> cloud_euler_pose = getEulerPoseFromMatrix(cloud_tf_vec[i].cast<double>());
      Eigen::Matrix4f pos_tf = _pos_tf_vec[0].inverse()*_pos_tf_vec[i];
      std::vector<double> pos_euler_pose = getEulerPoseFromMatrix(pos_tf.cast<double>());

      pos_file << pos_euler_pose[3] << " " << pos_euler_pose[4] << " " << pos_euler_pose[5]
               << " " << RAD2DEG(pos_euler_pose[0]) << " " << RAD2DEG(pos_euler_pose[1]) << " " << RAD2DEG(pos_euler_pose[2]) << std::endl;
      cloud_file << cloud_euler_pose[3] << " " << cloud_euler_pose[4] << " " << cloud_euler_pose[5]
               << " " << RAD2DEG(cloud_euler_pose[0]) << " " << RAD2DEG(cloud_euler_pose[1]) << " " << RAD2DEG(cloud_euler_pose[2]) << std::endl;
    }

    pos_file.close();
    cloud_file.close();
  }

  return cloud_tf_vec;
}

std::vector<double> BaseCalibration::calRotationResult(const Eigen::Matrix4d& _straight_tf, const Eigen::Matrix4d& _circle_tf,
  const bool _anticlockwise, const bool _forward) {

  Eigen::Matrix3d circle_rmat = _circle_tf.block<3, 3>(0, 0);
  Eigen::Matrix3d rmat_src_tar = circle_rmat.transpose();
  Eigen::AngleAxisd r_src_tar(rmat_src_tar);

  Eigen::Vector3d rvec_src_tar = r_src_tar.axis();
  Eigen::Vector3d tvec_src_tar =
      _straight_tf.inverse().block<3, 1>(0, 3).normalized();

  rvec_src_tar = _anticlockwise ? rvec_src_tar : rvec_src_tar * -1.0;
  tvec_src_tar = _forward ? tvec_src_tar : tvec_src_tar * -1.0;

  Eigen::Vector3d cross_vec = rvec_src_tar.cross(tvec_src_tar);

  Eigen::Matrix3d Rvs;
  Rvs.row(0) = tvec_src_tar.transpose();
  Rvs.row(1) = cross_vec.transpose();
  Rvs.row(2) = rvec_src_tar.transpose();
  Eigen::Matrix4d Rvs4d;
  Rvs4d.setIdentity();
  Rvs4d.block<3, 3>(0, 0) = Rvs;
  auto Rvs_inv = Rvs4d.inverse();
  auto eular = getEulerPoseFromMatrix(Rvs_inv);
  std::string pose_str = "eular pose: ";
  for (auto it : eular) {
    pose_str += (std::to_string(it) + ", ");
  }
  RCLCPP_DEBUG(ros_node_->get_logger(), "calRotationResult Pose %s ", pose_str.c_str());
  return eular;
}


void BaseCalibration::handEyeCali(const std::string _pose_folder_path)
{
  std::string arm_pose_path = _pose_folder_path + "/arm_pose.txt";
  std::string cloud_pose_path = _pose_folder_path + "/cloud_pose.txt";

  std::vector<Eigen::Matrix4d> arm_pose_vec = loadPoseInfo(arm_pose_path);
  std::vector<Eigen::Matrix4d> cloud_pose_vec = loadPoseInfo(cloud_pose_path);

  Eigen::Matrix4d cali_tf;
  if(handEyeCalibrationNavy(arm_pose_vec, cloud_pose_vec, cali_tf))
  {
    std::vector<double> lidar_to_arm_pose = getEulerPoseFromMatrix(cali_tf);
    auto quaterion = getQuaterionFromEulerPose(lidar_to_arm_pose);

    std::cout << "Q w         " << quaterion.w()<< std::endl;
    std::cout << "Q x         " << quaterion.x()<< std::endl;
    std::cout << "Q y         " << quaterion.y()<< std::endl;
    std::cout << "Q z         " << quaterion.z()<< std::endl;

    std::cout << "Eular roll  " << RAD2DEG(lidar_to_arm_pose[0])<< std::endl;
    std::cout << "Eular pitch " << RAD2DEG(lidar_to_arm_pose[1])<< std::endl;
    std::cout << "Eular yaw   " << RAD2DEG(lidar_to_arm_pose[2]) << std::endl;
    std::cout << "Eular x     " << lidar_to_arm_pose[3] << std::endl;
    std::cout << "Eular y     " << lidar_to_arm_pose[4] << std::endl;
    std::cout << "Eular z     " << lidar_to_arm_pose[5] << std::endl;
    }
}

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense