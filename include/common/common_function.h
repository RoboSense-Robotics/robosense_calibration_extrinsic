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

#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H

#include <dirent.h>

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <iostream>
#include <math.h>
#include <fstream>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "common_variable.h"
#include <pcl/octree/octree_pointcloud_pointvector.h>

namespace robosense {
namespace calib {
/*
 * revert the nesting vector
 */
template <typename T>
std::vector<std::vector<T>> revertVector(
    const std::vector<std::vector<T>>& matrix) {
  std::vector<std::vector<T>> matrix_rev(matrix[0].size(), std::vector<T>());
  for (size_t i = 0; i < matrix.size(); i++) {
    if (matrix[i].size() == matrix[0].size()) {
      for (size_t j = 0; j < matrix[0].size(); j++) {
        matrix_rev[j].push_back(matrix[i][j]);
      }
    } else {
      int temp = matrix[0].size() - matrix[i].size();
      for (size_t j = 0; j < matrix[0].size() - temp; j++) {
        matrix_rev[j].push_back(matrix[i][j]);
      }
    }
  }
  return matrix_rev;
}

template <typename T>
std::vector<size_t> sortIndexes(const std::vector<T>& v,
                                const std::string& method = "larger") {
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  if (method == "larger")
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
  else if (method == "smaller")
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });
  else if (method == "abs_larger")
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return abs(v[i1]) < abs(v[i2]); });

  return idx;
}

std::string getFileSuffix(const std::string& file_name);
std::vector<std::string> getFilesInDirectory(const std::string& directory);
std::vector<std::string> getJpgFilesInDirectory(const std::string& directory);
std::string formatNumberString(const int num);
std::string getCurrentTime();

void mkPath(const std::string &path);

/*
 * Transition between 4*4 Transform-Matrix and 6-dof(r-t) Pose-Vector(Euler or
 * Axis)
 */
std::vector<double> getEulerPoseFromMatrix(const Eigen::Matrix4d& trans_matrix);
std::vector<double> getAxisPoseFromMatrix(const Eigen::Matrix4d& trans_matrix);
Eigen::Matrix4d getMatrixFromTransAndQuaterion(const std::vector<double>& trans_pose, Eigen::Quaterniond& quaternion_pose);
Eigen::Matrix4d getMatrixFromEulerPose(const std::vector<double>& pose);
Eigen::Matrix4d getMatrixFromAxisPose(const std::vector<double>& pose);
std::vector<double> getEulerPoseFromVector(const std::vector<double>& _rvec,
                                           const std::vector<double>& _tvec,
                                           const Eigen::Matrix4d& _base_mat,
                                           Eigen::Matrix4d& _mat);
Eigen::Quaterniond getQuaterionFromEulerPose(const std::vector<double>& pose);
std::vector<double> getEulerFromQuaternion(Eigen::Quaterniond &quaternion_pose);

std::vector<double> transEularAngleToAxisAngle(
    const std::vector<double>& _pose_vec);
std::vector<double> transAxisAngleToEularAngle(
    const std::vector<double>& _pose);

bool checkDisMatch(const Eigen::Matrix4d &_A, const Eigen::Matrix4d &_B, const double _dis_thresh);

std::vector<Eigen::Matrix4d> loadPoseInfo(std::string pose_path);

Eigen::Matrix3d handEyeCalibrationR(const std::vector<Eigen::Matrix4d> &_A, const std::vector<Eigen::Matrix4d> &_B);

bool handEyeCalibrationNavy(const std::vector<Eigen::Matrix4d> &_A, const std::vector<Eigen::Matrix4d> &_B,
                            Eigen::Matrix4d &_T_B2A);

void octreeFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_cloud,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr& _output_cloud,
                  const float& _filter_leaf_size);

int getLeafCenterPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_cloud, std::vector<int>& _indices);

}  // namespace calib
}  // namespace robosense
#endif
