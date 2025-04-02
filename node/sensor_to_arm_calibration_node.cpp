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

#include <rclcpp/rclcpp.hpp>
#include "calibration/robotic_arm_calibration.h"

int main(int argc, char** argv) {
  std::string config_path =
      "/config/lidar_to_base_calib_config.yaml";
  if (argc >= 2) {
    config_path = argv[1];
  }

  rclcpp::init(argc, argv);

  robosense::calib::factory_calibration::RoboticArmCalibration robotic_arm_calib;

  robotic_arm_calib.startCalib(config_path);

  return 0;
}
