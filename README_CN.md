# CALIBRATION_EXTINSIC

[English Version](README.md)

## 1. 介绍

### 1.1 RS LiDAR

该工程适用于标定 速腾聚创激光 **雷达与车体** 的坐标关系

### 1.2 标定输入输出

**CALIBRATION_EXTINSIC** 的输入为

- 一组沿着直线匀速行驶的LiDAR数据
- 一组绕固定轴匀速旋转的LiDAR数据

**CALIBRATION_EXTINSIC** 的输出为

- LiDAR 到车体中心的 安装角度信息(四元数)

## 2. 数据样例

[demo数据包下载](https://cdn.robosense.cn/AC_wiki/calibration_extrinsic.zip)

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/straight_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"直线行驶数据" 数据集. 左图: 原始图像, 右图: 点云数据</p>
</div>

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/circle_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"曲线行驶数据" 数据集. 左图: 原始图像, 右图: 点云数据</p>
</div>

## 3. 依赖

### 3.1 ROS2

此项目基于 `ros2 humble`进行开发测试

根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

## 4. 安装编译

下载仓库：

您可以创建一个新的文件夹或进入您现有的 `ros2` 工作空间，执行以下命令将代码拉取到工作空间内

```bash
git clone https://github.com/RoboSense-Robotics/robosense_calibration_extrinsic.git -b main
```

在您的工作空间下执行以下命令来编译安装 `robosense_calibration_extrinsic`

```bash
colcon build --symlink-install
```

编译安装完成后，推荐刷新一下工作空间的 `bash profile`，确保组件功能正常

```bash
source install/setup.bash
```

## 4. 运行

### 4.1 重要参数

编辑 `config/lidar_to_base_calib_config.yaml` 来设置一些重要参数：

- LIDAR:
  - straight_lidar_topic: 直线行驶的 LiDAR 话题
  - circle_lidar_topic: 曲线行驶的 LiDAR 话题
  - cloud_size: 标定使用的点云帧数
  - lidar_calibration_file: LiDAR 的相关标定参数文件的绝对路径 （需要读取 LiDAR 和 Sensor 的 坐标关系）
- BASE:
  - lidar_registration_config_file: NDT配准相关参数的绝对路径
  - anticlockwise: 对于旋转数据，如果LiDAR是绕顺时针旋转，则设置为 true； 如果LiDAR是绕逆时针旋转，则设置为 false
  - forward: 对于直行数据，如果LiDAR是向前行驶，则设置为 true； 如果LiDAR是倒退直行，则设置为 false
  - save_path: 用于存储 配准的过程文件的绝对路径，如果设置为 ''， 则不存储

编辑 `config/default_config/registration_config.yaml` 来设置一些配准参数（推荐使用默认值）：

- NDT:
  - LeafSize: 点云体素滤波的参数
  - StepSize: 迭代步长
  - Resolution: 分辨率
  - MaxIteration: 最大迭代次数
  - Epsilon: 精度阈值

在此之后，您可以直接在数据集上运行**robosense_calibration_extrinsic**。

### 4.2 在数据集运行

从速腾官方下载demo数据集， 共包含 **2** 个rosbag

- 启动程序 (需要输入 ```lidar_to_base_calib_config.yaml```参数的绝对路径)

```bash
ros2 run robosense_calibration_extrinsic sensor_to_body_calibration_node "XXX/robosense_calibration_extrinsic/config/lidar_to_base_calib_config.yaml"
```

- 播放 直行数据

```bash
ros2 bag play xxxx.db3 --remap /rs_lidar/points:=/straight/rs_lidar/points # demo数据已经进行过remap，无需再次转换
```

- 播放 旋转数据

```bash
ros2 bag play xxxx.db3 --remap /rs_lidar/points:=/circle/rs_lidar/points # demo数据已经进行过remap，无需再次转换
```

（数据集中录制的topic在不同域名下，可同时播放数据，互不干扰）
