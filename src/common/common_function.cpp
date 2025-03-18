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

#include "common/common_function.h"

#include <ctime>

#include <ceres/rotation.h>
namespace robosense {
namespace calib {

std::string getFileSuffix(const std::string& file_name) {
  if (file_name.empty()) {
    return file_name;
  }
  int pos = file_name.find_last_of(".");
  return file_name.substr(pos + 1);
}

std::vector<std::string> getFilesInDirectory(const std::string& directory) {
  std::vector<std::string> files;
  DIR* dir;
  struct dirent* entry;
  // 打开目录
  dir = opendir(directory.c_str());
  if (dir) {
    // 遍历目录中的文件
    while ((entry = readdir(dir)) != nullptr) {
      // 获取文件名
      std::string filename = entry->d_name;
      std::string suffix = getFileSuffix(filename);
      if (suffix == "png" || suffix == "jpeg" || suffix == "jpg") {
        files.push_back(filename);
      }
    }
    // 关闭目录
    closedir(dir);
  }
  return files;
}

std::vector<std::string> getJpgFilesInDirectory(const std::string& directory) {
  std::vector<std::string> jpgFiles;
  DIR* dir;
  struct dirent* entry;

  // 打开目录
  dir = opendir(directory.c_str());
  if (dir) {
    // 遍历目录中的文件
    while ((entry = readdir(dir)) != nullptr) {
      // 获取文件名
      std::string filename = entry->d_name;

      // 判断文件名是否以 ".jpg" 结尾
      if (filename.size() >= 4 &&
          filename.substr(filename.size() - 4) == ".jpg") {
        // 添加到结果向量中
        jpgFiles.push_back(filename);
      }
    }
    // 关闭目录
    closedir(dir);
  }
  return jpgFiles;
}

std::string formatNumberString(const int num) {
  std::string str = std::to_string(num);
  return (num < 10 ? "0" + str : str);
}

std::string getCurrentTime() {
  std::time_t now = std::time(nullptr);
  std::tm* local_time = std::localtime(&now);

  std::string year_str = formatNumberString(local_time->tm_year + 1900);
  std::string mon_str = formatNumberString(local_time->tm_mon + 1);
  std::string mday_str = formatNumberString(local_time->tm_mday);
  std::string hour_str = formatNumberString(local_time->tm_hour);
  std::string min_str = formatNumberString(local_time->tm_min);
  std::string sec_str = formatNumberString(local_time->tm_sec);
  return year_str + mon_str + mday_str + "_" + hour_str + min_str + sec_str;
}

void mkPath(const std::string& path) {
  std::string mkdir = "mkdir -p " + path;
  system(mkdir.c_str());
}

std::vector<double> getEulerPoseFromMatrix(
    const Eigen::Matrix4d& trans_matrix) {
  std::vector<double> pose;
  Eigen::Matrix3d rotation_matrix = trans_matrix.block(0, 0, 3, 3);
  // roll pitch yaw order(Rz*Ry*Rx)
  Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(2, 1, 0);

  pose.push_back(euler_angle(2));
  pose.push_back(euler_angle(1));
  pose.push_back(euler_angle(0));
  pose.push_back(trans_matrix(0, 3));
  pose.push_back(trans_matrix(1, 3));
  pose.push_back(trans_matrix(2, 3));
  return pose;
}

std::vector<double> getAxisPoseFromMatrix(const Eigen::Matrix4d& trans_matrix) {
  std::vector<double> pose;
  double rotation[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rotation[i * 3 + j] = trans_matrix(j, i);
    }
  }

  double ceres_r[3]{0.0, 0.0, 0.0};
  double ceres_t[3]{trans_matrix(0, 3), trans_matrix(1, 3), trans_matrix(2, 3)};
  ceres::RotationMatrixToAngleAxis(rotation, ceres_r);
  pose.push_back(ceres_r[0]);
  pose.push_back(ceres_r[1]);
  pose.push_back(ceres_r[2]);
  pose.push_back(ceres_t[0]);
  pose.push_back(ceres_t[1]);
  pose.push_back(ceres_t[2]);
  return pose;
}

Eigen::Quaterniond getQuaterionFromEulerPose(const std::vector<double>& pose)
{
      // 创建一个四元数对象
    Eigen::Quaterniond quaternion;

    // 使用 Eigen 的 AngleAxis 类将欧拉角转换为四元数
    quaternion = Eigen::AngleAxisd(pose[0], Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(pose[1], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ());

    return quaternion;
}

Eigen::Matrix4d getMatrixFromTransAndQuaterion(const std::vector<double>& trans_pose, Eigen::Quaterniond& quaternion_pose)
{
  Eigen::Matrix3d rotation_matrix = quaternion_pose.toRotationMatrix();

  Eigen::Matrix4d trans_matrix;
  trans_matrix.setIdentity();
  trans_matrix.block(0, 0, 3, 3) = rotation_matrix;

  trans_matrix(0, 3) = trans_pose[0];
  trans_matrix(1, 3) = trans_pose[1];
  trans_matrix(2, 3) = trans_pose[2];
  return trans_matrix;
}

Eigen::Matrix4d getMatrixFromEulerPose(const std::vector<double>& pose) {
  Eigen::Matrix4d trans_matrix;
  // yaw pitch roll order
  Eigen::Vector3d euler_angle;
  euler_angle << pose[0], pose[1], pose[2];
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;

  trans_matrix.setIdentity();
  trans_matrix.block(0, 0, 3, 3) = rotation_matrix;

  trans_matrix(0, 3) = pose[3];
  trans_matrix(1, 3) = pose[4];
  trans_matrix(2, 3) = pose[5];
  return trans_matrix;
}

Eigen::Matrix4d getMatrixFromAxisPose(const std::vector<double>& pose) {
  Eigen::Matrix4d trans_matrix;
  double ceres_r[3]{pose[0], pose[1], pose[2]};
  double ceres_t[3]{pose[3], pose[4], pose[5]};
  double rotation[9];
  ceres::AngleAxisToRotationMatrix<double>(ceres_r, rotation);
  trans_matrix.setIdentity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      trans_matrix(j, i) = rotation[i * 3 + j];
    }
  }
  trans_matrix(0, 3) = ceres_t[0];
  trans_matrix(1, 3) = ceres_t[1];
  trans_matrix(2, 3) = ceres_t[2];
  return trans_matrix;
}

std::vector<double> transEularAngleToAxisAngle(
    const std::vector<double>& _pose) {
  Eigen::Matrix4d trans_matrix = getMatrixFromEulerPose(_pose);
  return getAxisPoseFromMatrix(trans_matrix);
}

std::vector<double> transAxisAngleToEularAngle(
    const std::vector<double>& _pose) {
  Eigen::Matrix4d trans_matrix = getMatrixFromAxisPose(_pose);
  return getEulerPoseFromMatrix(trans_matrix);
}

std::vector<double> getEulerPoseFromVector(const std::vector<double>& _rvec,
                                           const std::vector<double>& _tvec,
                                           const Eigen::Matrix4d& _base_mat,
                                           Eigen::Matrix4d& _mat) {
  std::vector<double> axis_pose = {
      static_cast<double>(_rvec[0]), static_cast<double>(_rvec[1]),
      static_cast<double>(_rvec[2]), static_cast<double>(_tvec[0]),
      static_cast<double>(_tvec[1]), static_cast<double>(_tvec[2]),
  };

  auto trans_mat = getMatrixFromAxisPose(axis_pose);

  // auto pose = getEulerPoseFromMatrix(trans_mat);
  _mat = _base_mat.inverse() * trans_mat.inverse();
  auto pose = getEulerPoseFromMatrix(_mat);
  return pose;
}

}  // namespace calib
}  // namespace robosense
