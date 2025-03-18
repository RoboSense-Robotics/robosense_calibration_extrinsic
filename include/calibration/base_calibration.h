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

#ifndef BASE_CALIBRATION_H
#define BASE_CALIBRATION_H

#include <string>
#include <vector>
#include <deque>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>

namespace robosense {
namespace calib {
namespace factory_calibration {
struct NDTRegistrationParam {
  std::vector<double> leaf_size;
  double step_size = 0;
  double resolution = 0;
  int max_iteration = 0;
  double epsilon = 0;
};
class BaseCalibration {
 private:

  bool anticlockwise_ = true;
  bool forward_ = true;
  std::string calculating_type = "";
  std::string save_path_;
  std::string registration_config_path_;
  NDTRegistrationParam ndt_param_;

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter_;
  // pcl::VoxelGrid<pcl::PointXYZI> approximate_voxel_filter_;

  rclcpp::Node::SharedPtr ros_node_;

  bool loadRegistrationConfig(const std::string _config_path);

  void registrationParamSet();

  pcl::PointCloud<pcl::PointXYZI>::Ptr getNDTMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr &_source_cloud,
                                                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &_target_cloud,
                                                   const Eigen::Matrix4f &_init_tf, Eigen::Matrix4f &_match_tf);

  pcl::PointCloud<pcl::PointXYZI>::Ptr getFilterCloud(pcl::PointCloud<pcl::PointXYZI> & input_cloud);

  Eigen::Matrix4d getPointCloudFirstToLastRegistration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _straight_cloud_vec);

  std::vector<double> calRotationResult(const Eigen::Matrix4d &_straight_tf,
                                        const Eigen::Matrix4d &_circle_tf,
                                        const bool _anticlockwise,
                                        const bool _forward);

public:
  BaseCalibration();
  ~BaseCalibration();
  bool loadConfig(const std::string _config_path, rclcpp::Node::SharedPtr _ros_node);
  std::vector<double> executeCalibration(std::vector<pcl::PointCloud<pcl::PointXYZI>> & _straight_cloud_vec,
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & _circle_cloud_vec);
};

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
#endif
