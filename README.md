# CALIBRATION_EXTINSIC

[中文文档](README_CN.md)

## 1. Introduction

### 1.1 RS LiDAR

This repository supports two functions . Recommended to run on x86 system.

- adapts the caliration from LiDAR To Car BaseLink
- adapts the caliration from LiDAR To Robotic Arm

### 1.2 Calibration I/O

#### 1.2.1 LiDAR To Car BaseLink Calibration

INPUT DATA

- A set of LiDAR data captured during low-speed linear movement (travel distance: ≥5m, speed: ≤1m/s) on horizontal ground
- A set of LiDAR data captured during constant-speed rotation (angular displacement: ≥45°, speed: ≤1m/s) on horizontal ground

OUPUT RESULT

- LiDAR to BaseLink (Quaterion)

#### 1.2.2 LiDAR to Robotic Arm Calibration

INPUT DATA

- A set of LiDAR data and Robotic Arm Pose traveling along a series of poses

Motion captured with reference to the robotic arm's end-effector frame:

- Significant translational displacements along x, y, and z axes(travel distance: ≥20cm)
- Complete angular rotations about roll, pitch, and yaw axes(angular displacement: ≥20°)
- Post-motion static maintenance: ≥4s stationary state after each movement

OUTPUT RESULT

- Lidar to Robotic Arm Pose (Translation and Quaterion)

## 2. Dependency

This project is developed and tested based on ROS 2 Humble.

Follow the specified content in the [official tutorial](https://fishros.org/doc/ros2/humble/Installation.html) for your operating system.

## 3. Install and Build

You can create a new folder or go into your existing `ros2` workspace and execute the following command to clone the code into the workspace.

```bash
git clone https://github.com/RoboSense-Robotics/robosense_calibration_extrinsic.git -b main
```

Execute the following commands in your workspace to compile and install `robosense_calibration_extrinsic`

```bash
colcon build --symlink-install
```

After compiling and installing, it is recommended to refresh the 'bash profile' in the workspace to ensure that the components function properly

```bash
source install/setup.bash
```

## 4. Parameters Description

### 4.1 LiDAR to Car BaseLink parameters

Edit `config/lidar_to_base_calib_config.yaml` to set the below parameters:

- LIDAR:
  - `straight_lidar_topic`: Straight LiDAR Topic
  - `circle_lidar_topic`: Circle LiDAR Topic
  - `cloud_size`: The threshold of point cloud frames used for calibration
  - `lidar_calibration_file`: Absolute path of LiDAR calibration file （ Lidar_to_Sensor）
- BASE:
  - `lidar_registration_config_file`: Absolute path of NDT registration related parameters
  - `anticlockwise`: For Circle data, if LiDAR rotates clockwise, set it to true; If LiDAR rotates counterclockwise, set to false
  - `forward`: For Straight data, if LiDAR is driving forward, set it to true; If LiDAR is reverse straight, set to false
  - `save_path`: The absolute path used to store the registration process file, if set to '', will not be stored

### 4.2 LiDAR to Robotic Arm parameters

Edit `config/lidar_to_arm_calib_config.yaml` to set the below parameters:

- LiDAR:
  - `pos_cmd_topic`: Robotic Arm Pose Topic
  - `lidar_topic`: LiDAR topic
  - `pos_stable_time_thresh`: Pose Stable Frame Threshold (the stable frame num threshold obtained when the Robotic Arm is still, used to determine whether data has been successfully obtained in a still state)
  - `pos_ok_time_thresh`: Effective data threshold (the threshold for the times stable data is obtained, When the threshold is reached,the program completes the data collection process and starts the calibration)
  - `lidar_calibration_file`: Absolute path of LiDAR calibration file （ Lidar_to_Sensor）

- BASE:
  - `lidar_registration_config_file`: Absolute path of NDT registration related parameters
  - `anticlockwise`: For Circle data, if LiDAR rotates clockwise, set it to true; If LiDAR rotates counterclockwise, set to false
  - `forward`: For Straight data, if LiDAR is driving forward, set it to true; If LiDAR is reverse straight, set to false
  - `save_path`: The absolute path used to store the registration process file, if set to '', will not be stored

### 4.3 NDT Registration parameters

Edit `config/default_config/registration_config.yaml` to set the registration parameters

- NDT:
  - `LeafSize`: Parameters of point cloud voxel filtering
  - `StepSize`: Iteration Step Size
  - `Resolution`: Resolution
  - `MaxIteration`: Max Iteration threshold
  - `Epsilon`: Epsilon threshold

After setting the appropriate topic name and parameters, you can directly run **robosense_calibration_extrinsic** on the dataset.

## 5 Run

### 5.1 LiDAR to Car BaseLink

- Collecte Data
  Activate the lidar driver, control the wheeled mobile robot to move, and collect two sets of lidar data(as shown in the example data diagram below, or download the demo data):

  - A set of LiDAR data captured during low-speed linear movement (travel distance: ≥5m, speed: ≤1m/s) on horizontal ground

  - A set of LiDAR data captured during constant-speed rotation (angular displacement: ≥45°, speed: ≤1m/s) on horizontal ground

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/straight_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"Straight" Dataset. Left: Image, Right: LiDAR</p>
</div>

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/circle_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"Circle" Dataset. Left: Image, Right: LiDAR</p>
</div>

- Run (need to input the absolute path of ```lidar_to_base_calib_config.yaml```)

```bash
ros2 run robosense_calibration_extrinsic sensor_to_body_calibration_node "XXX/robosense_calibration_extrinsic/config/lidar_to_base_calib_config.yaml"
```

- play Straight data

```bash
ros2 bag play xxxx.db3 --remap /rs_lidar/points:=/straight/rs_lidar/points # The demo data has already been remapped; no additional conversion is required.
```

- play Circle data

```bash
ros2 bag play xxxx.db3 --remap /rs_lidar/points:=/circle/rs_lidar/points # The demo data has already been remapped; no additional conversion is required.
```

- Reference
  - [demo data](https://cdn.robosense.cn/AC_wiki/calibration_extrinsic.zip)

#### 5.2 LiDAR to Robotic Arm

- Run (need to input the absolute path of ```lidar_to_arm_calib_config.yaml```)

  ```bash

  ros2 run robosense_calibration_extrinsic sensor_to_arm_calibration_node "XXX/robosense_calibration_extrinsic/config/lidar_to_arm_calib_config.yaml"

  ```

- Collecte Data
  Activate the lidar driver, control the robotic arm to move in different directions:

  - Motion captured with reference to the robotic arm's end-effector frame, the robotic arm's motion trajectory should cover all degrees of freedom (x, y, z, roll, pitch, yaw) as comprehensively as possible(travel distance: ≥20cm, angular displacement: ≥20°).
  
  - After each movement, keep stationary for ​4 seconds.

- Reference
  - [example video](https://cdn.robosense.cn/AC_wiki/sensor_to_mechanical_arm.mp4)
  - [demo data](https://cdn.robosense.cn/AC_wiki/sensor_to_arm_calib.zip)
