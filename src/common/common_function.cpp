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
#include <cmath> 
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

std::vector<double> getEulerFromQuaternion(Eigen::Quaterniond &quaternion_pose)
{
  Eigen::Matrix3d rotation_matrix = quaternion_pose.toRotationMatrix();
  Eigen::Matrix4d trans_matrix;
  trans_matrix.setIdentity();
  trans_matrix.block(0, 0, 3, 3) = rotation_matrix;

  std::vector<double> euler_pose = getEulerPoseFromMatrix(trans_matrix);
  return euler_pose;
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
  // yaw pitch roll order
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd(pose[0], Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd(pose[1], Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;

  return Eigen::Quaterniond(rotation_matrix).normalized();
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

std::vector<Eigen::Matrix4d> loadPoseInfo(std::string pose_path)
{
    std::vector<Eigen::Matrix4d> pose_vec;

    std::ifstream file(pose_path);

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file " << pose_path << std::endl;
        return pose_vec;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        // 跳过空行和注释行（以#开头）
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::vector<double> elements;
        double val;
        // 解析每行中的 6个数值
        while (iss >> val) {
            elements.push_back(val);
        }

        // 验证数据数量是否正确
        if (elements.size() != 6) {
            std::cerr << "Error: Line " << line_num 
                      << " contains " << elements.size() 
                      << " elements (expected 6)" << std::endl;
            continue;
        }

        std::vector<double> eular_pose;
        eular_pose.push_back( elements[3]*M_PI / 180.0);
        eular_pose.push_back( elements[4]*M_PI / 180.0);
        eular_pose.push_back( elements[5]*M_PI / 180.0);
        eular_pose.push_back( elements[0]);
        eular_pose.push_back( elements[1]);
        eular_pose.push_back( elements[2]);

        Eigen::Matrix4d tf = getMatrixFromEulerPose(eular_pose);

        pose_vec.push_back(tf);
    }

    file.close();

    return pose_vec;
}

bool checkDisMatch(const Eigen::Matrix4d& _A, const Eigen::Matrix4d& _B, const double _dis_thresh)
{
    double a_dis = std::sqrt(_A(0,3)*_A(0,3) + _A(1,3)*_A(1,3) + _A(2,3)*_A(2,3));
    double b_dis = std::sqrt(_B(0,3)*_B(0,3) + _B(1,3)*_B(1,3) + _B(2,3)*_B(2,3));
    double dif = std::fabs(a_dis - b_dis);

    std::vector<double> a_pose = getEulerPoseFromMatrix(_A);
    std::vector<double> b_pose = getEulerPoseFromMatrix(_B);

    // std::cout << "----------------------" << std::endl;

    // std::cout << "A " << a_pose[3] << " " << a_pose[4] << " " << a_pose[5] << " " << a_pose[0] * 180 / M_PI << " " << a_pose[1] * 180 / M_PI << " " << a_pose[2] * 180 / M_PI << std::endl;
    // std::cout << "B " << b_pose[3] << " " << b_pose[4] << " " << b_pose[5] << " " << b_pose[0] * 180 / M_PI << " " << b_pose[1] * 180 / M_PI << " " << b_pose[2] * 180 / M_PI << std::endl;

    // std::cout << "a_dis " << a_dis << " b_dis " << b_dis << " dif = " << dif << std::endl;

    if (dif < _dis_thresh)
    {
      return true;
    }

    return false;
}

Eigen::Matrix3d handEyeCalibrationR(const std::vector<Eigen::Matrix4d>& _A, const std::vector<Eigen::Matrix4d>& _B)
{
  std::vector<Eigen::Matrix4d> delta_a_delta_vec;
  std::vector<Eigen::Matrix4d> delta_b_delta_vec;

  double angle_rad_thresh = DEG2RAD(1);

  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  for (std::size_t i = 1; i < _A.size();i++)
  {
      Eigen::Matrix4d delta_a_transform = _A[i-1].inverse() * _A[i];
      Eigen::Matrix4d delta_b_transform = _B[i-1].inverse() * _B[i];

      const Eigen::Matrix3d delta_R_A = delta_a_transform.block<3, 3>(0, 0);
      const Eigen::Matrix3d delta_R_B = delta_b_transform.block<3, 3>(0, 0);
      Eigen::AngleAxis<double> delta_angleAxis_A(delta_R_A);
      Eigen::AngleAxis<double> delta_angleAxis_B(delta_R_B);
      Eigen::Matrix<double, 3, 1> delta_rotationVector_A = delta_angleAxis_A.angle() * delta_angleAxis_A.axis();
      Eigen::Matrix<double, 3, 1> delta_rotationVector_B = delta_angleAxis_B.angle() * delta_angleAxis_B.axis();
      double angle_dif = std::fabs(delta_angleAxis_A.angle() - delta_angleAxis_B.angle());
      // std::cout << "i------------ " << i<< std::endl;

      std::vector<double> delta_a_pose = getEulerPoseFromMatrix(delta_a_transform);
      std::vector<double> delta_b_pose = getEulerPoseFromMatrix(delta_b_transform);

      // std::cout << "Delta A " << delta_a_pose[3] << "   \t" << delta_a_pose[4] << "   \t" << delta_a_pose[5] << "   \t" << RAD2DEG(delta_a_pose[0]) << "    \t" <<  RAD2DEG(delta_a_pose[1]) << "   \t" <<  RAD2DEG(delta_a_pose[2]) << std::endl;
      // std::cout << "Delta B " << delta_b_pose[3] << "   \t" << delta_b_pose[4] << "   \t" << delta_b_pose[5] << "   \t" << RAD2DEG(delta_b_pose[0]) << "    \t" <<  RAD2DEG(delta_b_pose[1]) << "   \t" <<  RAD2DEG(delta_b_pose[2]) << std::endl;
      if (angle_dif > angle_rad_thresh)
      {
        continue;
      }
      // std::cout << "a " << delta_angleAxis_A.angle() * 180 / M_PI << " b " << delta_angleAxis_B.angle() * 180 / M_PI << " delta " << angle_dif * 180 / M_PI << std::endl;
      M += delta_rotationVector_B * delta_rotationVector_A.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_           = svd.matrixV() * svd.matrixU().transpose();

  if (R_.determinant() < 0)
  {
    Eigen::Matrix3d trans;
    trans << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    R_ = svd.matrixV() * trans * svd.matrixU().transpose();
  }

  return R_;
}

bool handEyeCalibrationNavy(const std::vector<Eigen::Matrix4d>& _A, const std::vector<Eigen::Matrix4d>& _B,
  Eigen::Matrix4d& _T_B2A)
{
  if (_A.size() != _B.size() || _A.size() <= 3)
  {
    return false;
  }

  Eigen::Matrix3d R_ = handEyeCalibrationR(_A, _B);

  _T_B2A.template block<3, 3>(0, 0) = R_;

  double dis_thresh = 0.05;
  int dis_ok_data_size = 0;
  std::vector<bool> dis_ok_vec;
  for (std::size_t i = 1; i < _A.size(); i++)
  {
    Eigen::Matrix4d delta_a_transform = _A[i-1].inverse() * _A[i];
    Eigen::Matrix4d delta_b_transform = _B[i-1].inverse() * _B[i];

    if (checkDisMatch(delta_a_transform, delta_b_transform, dis_thresh) == false)
    {
      dis_ok_vec.push_back(false);
      continue;
    }

    dis_ok_vec.push_back(true);
    dis_ok_data_size++;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> C(3 * dis_ok_data_size, 3);
  Eigen::Matrix<double, Eigen::Dynamic, 1> d(3 * dis_ok_data_size);

  int effect_data_count = 0;
  for (std::size_t i = 1; i < _A.size(); ++i)
  {
    if(dis_ok_vec[i] == false)
    {
      continue;
    }

    const Eigen::Matrix<double, 3, 3> R_A = _A[i].block<3, 3>(0, 0);
    const Eigen::Matrix<double, 3, 1> t_A = _A[i].block<3, 1>(0, 3);
    const Eigen::Matrix<double, 3, 1> t_B = _B[i].block<3, 1>(0, 3);

    C.block<3, 3>(3 * effect_data_count, 0) = R_A - Eigen::Matrix<double, 3, 3>::Identity();
    d.segment<3>(3 * effect_data_count) = R_ * t_B - t_A;
    effect_data_count++;
  }

  const Eigen::Matrix<double, 3, 1> t = C.colPivHouseholderQr().solve(d);
  _T_B2A.block<3, 1>(0, 3) = t;

  return true;
}

int getLeafCenterPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_cloud, std::vector<int>& _indices)
{
  float ave_x = 0;
  float ave_y = 0;
  float ave_z = 0;

  int size      = _indices.size();
  int count_num = 0;
  for (int i = 0; i < size; i++)
  {
    pcl::PointXYZI p = _input_cloud->points.at(_indices[i]);

    if (pcl::isFinite(p))
    {
      ave_x += p.x;
      ave_y += p.y;
      ave_z += p.z;
      count_num++;
    }
  }
  ave_x = (float)(ave_x / count_num);
  ave_y = (float)(ave_y / count_num);
  ave_z = (float)(ave_z / count_num);

  float closest_dis_2 = 1000;
  int closest_index   = -1;

  int indice_size = _indices.size();
  for (int i = 0; i < indice_size; i++)
  {
    pcl::PointXYZI p = _input_cloud->points.at(_indices[i]);
    if (pcl::isFinite(p))
    {
      float cur_dis_x = p.x - ave_x;
      float cur_dis_y = p.y - ave_y;
      float cur_dis_z = p.z - ave_z;
      float cur_dis_2 = cur_dis_x * cur_dis_x + cur_dis_y * cur_dis_y + cur_dis_z * cur_dis_z;
      if (cur_dis_2 < closest_dis_2)
      {
        closest_dis_2 = cur_dis_2;
        closest_index = _indices[i];
      }
    }
  }

  return closest_index;
}

void octreeFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_cloud,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr& _output_cloud,
                  const float& _filter_leaf_size)
{
  _output_cloud->clear();

  if (_filter_leaf_size > 0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_input_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (std::size_t i = 0; i < _input_cloud->size(); i++)
    {
      pcl::PointXYZI cur_p = _input_cloud->points.at(i);

      if (pcl::isFinite(cur_p))
      {
        filter_input_cloud->points.push_back(cur_p);
      }
    }

    pcl::octree::OctreePointCloudPointVector<pcl::PointXYZI> octree(_filter_leaf_size);

    octree.setInputCloud(filter_input_cloud);
    octree.addPointsFromInputCloud();

    // boost::shared_ptr<const std::vector<int> > indices = octree.getIndices();
    std::vector<pcl::octree::OctreePointCloudPointVector<pcl::PointXYZI>::LeafContainer*> leafs;
    octree.serializeLeafs(leafs);
    for (auto& leaf : leafs)
    {
      // get all points' indices in current cell
      std::vector<int> indices;
      leaf->getPointIndices(indices);
      if (indices.size() > 0)
      {
        int center_index  = getLeafCenterPoint(filter_input_cloud, indices);
        pcl::PointXYZI cp = filter_input_cloud->points.at(center_index);
        _output_cloud->points.emplace_back(cp);
      }
    }
  }
  else
  {
    *_output_cloud += *_input_cloud;
  }
}

}  // namespace calib
}  // namespace robosense
