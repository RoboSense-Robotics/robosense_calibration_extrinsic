# CALIBRATION_EXTINSIC

[中文文档](README_CN.md)

## 1. Introduction

### 1.1 RS LiDAR

This repository adapts the caliration from LiDar To BaseLink

### 1.2 Calibration I/O

**CALIBRATION_EXTINSIC** INPUT DATA

- A set of LiDAR data traveling at a constant speed along a straight line
- A set of LiDAR data rotating uniformly around a fixed axis

**CALIBRATION_EXTINSIC** OUPUT RESULT

- LiDAR to BaseLink (Quaterion)

## 2. Data Demo

[Sample Data](https://cdn.robosense.cn/AC_wiki/calibration_extrinsic.zip)

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/straight_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"Straight" Dataset. Left: Image, Right: LiDAR</p>
</div>

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/circle_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"Circle" Dataset. Left: Image, Right: LiDAR</p>
</div>

## 3. Prerequisited

### 3.1 ROS2

Follow the specified content in the [official tutorial](https://fishros.org/doc/ros2/humble/Installation.html) for your operating system.

## 4. Install and Build

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

## 4. Run the Package

### 4.1 Important parameters

Edit `config/lidar_to_base_calib_config.yaml` to set the below parameters：

- LIDAR:
  - 'straight_lidar_topic': Straight LiDAR Topic
  - 'circle_lidar_topic': Circle LiDAR Topic
  - 'cloud_size': The threshold of point cloud frames used for calibration
  - lidar_calibration_file: Absolute path of LiDAR calibration file （ Lidar_to_Sensor）
- BASE:
  - 'lidar_registration_config_file': Absolute path of NDT registration related parameters
  - 'anticlockwise': For Circle data, if LiDAR rotates clockwise, set it to true; If LiDAR rotates counterclockwise, set to false
  - 'forward': For Straight data, if LiDAR is driving forward, set it to true; If LiDAR is reverse straight, set to false
  - 'save_path': The absolute path used to store the registration process file, if set to '', will not be stored

Edit `config/default_config/registration_config.yaml` to set the registration parameters

- NDT:
  - 'LeafSize': Parameters of point cloud voxel filtering
  - 'StepSize': Iteration Step Size
  - 'Resolution': Resolution
  - 'MaxIteration': Max Iteration threshold
  - 'Epsilon': Epsilon threshold

After setting the appropriate topic name and parameters, you can directly run **robosense_calibration_extrinsic** on the dataset.

### 4.2 Run on dataset

Download our collected demo data sets, containing **two** data sets

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
