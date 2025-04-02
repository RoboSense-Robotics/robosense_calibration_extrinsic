# CALIBRATION_EXTINSIC

[English Version](README.md)

## 1. 介绍

### 1.1 RS LiDAR

该工程支持以下两个功能, 推荐在 x86 系统下运行

- 适用于标定速腾聚创激光 **雷达与车体** 的坐标关系
- 适用于标定速腾聚创激光 **雷达与机械臂** 的坐标关系

### 1.2 标定输入输出

#### 1.2.1 雷达到车体标定的数据输入输出

输入为

- 一组在水平地面上沿着直线低速前进的LiDAR数据，移动距离不低于5m，移动速度不超过1m/s
- 一组在水平地面上匀速旋转的LiDAR数据，角度偏转不低于45°，移动速度不超过1m/s

输出为

- LiDAR到车体中心的安装角度信息(四元数)

#### 1.2.2 雷达到机械臂标定的数据输入输出

输入为

- 一组覆盖多个姿态的机械臂末端姿态信息及同步录制的LiDAR数据

运动轨迹说明：

- 以机械臂末端坐标系为参考，机械臂末端存在充分的x, y, z轴的平移变化，移动距离不低于20cm
- 以机械臂末端坐标系为参考，机械臂末端存在roll, pitch, yaw三个轴的角度旋转变化，角度偏转不低于20°
- 每次运动完成后，保持静止状态不低于4s

输出为

- LiDAR到机械臂末端的安装角度信息(四元数+平移量)

## 2. 依赖

此项目基于 `ros2 humble`进行开发测试

根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

## 3. 安装编译

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

## 4. 参数说明

### 4.1 雷达到车体标定参数

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

### 4.2 雷达到机械臂标定参数

编辑 `config/lidar_to_arm_calib_config.yaml` 来设置一些重要参数：

- LIDAR:
  - pos_cmd_topic: 机械臂末端姿态 话题
  - lidar_topic: LiDAR 话题
  - pos_stable_time_thresh: 姿态稳定帧数阈值（机械臂静止时，获得的稳定帧数阈值，用于判断是否成功获取一次静止状态下的数据 ）
  - pos_ok_time_thresh: 有效数据帧数阈值（获得静止状态数据的次数阈值，获取稳定数据帧数达到这个阈值后，程序完成数据收集过程，开始标定 ）
  - lidar_calibration_file: LiDAR 的相关标定参数文件的绝对路径 （需要读取 LiDAR 和 Sensor 的 坐标关系）
- BASE:
  - lidar_registration_config_file: NDT配准相关参数的绝对路径
  - save_path: 用于存储 配准的过程文件的绝对路径，如果设置为 ''， 则不存储

### 4.3 点云配准公共参数

编辑 `config/default_config/registration_config.yaml` 来设置一些配准参数（推荐使用默认值）：

- NDT:
  - LeafSize: 点云体素滤波的参数
  - StepSize: 迭代步长
  - Resolution: 分辨率
  - MaxIteration: 最大迭代次数
  - Epsilon: 精度阈值

在此之后，您可以直接在数据集上运行**robosense_calibration_extrinsic**。

## 5 运行

### 5.1 雷达到车体标定

- 采集数据
  启动雷达驱动，控制轮式移动机器人移动，并采集两组雷达数据(如下图数据样例所示，或下载demo数据)：
  
  - 一组在水平地面上沿着直线低速前进的LiDAR数据，移动距离不低于5m，移动速度不超过1m/s

  - 一组在水平地面上匀速旋转的LiDAR数据，角度偏转不低于45°，移动速度不超过1m/s
  
<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/straight_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"直线行驶数据" 数据集. 左图: 原始图像, 右图: 点云数据</p>
</div>

<div align="center">
    <img src="https://cdn.robosense.cn/AC_wiki/circle_data.gif" alt="mesh" />
    <p style="margin-top: 2px;">"曲线行驶数据" 数据集. 左图: 原始图像, 右图: 点云数据</p>
</div>

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

- 参考资源:
  - [demo数据](https://cdn.robosense.cn/AC_wiki/calibration_extrinsic.zip)

### 5.2 雷达到机械臂标定

- 启动程序 (需要输入 ```lidar_to_arm_calib_config.yaml```参数的绝对路径)

```bash
ros2 run robosense_calibration_extrinsic sensor_to_arm_calibration_node "XXX/robosense_calibration_extrinsic/config/lidar_to_arm_calib_config.yaml"
```

- 采集数据
  启动雷达驱动，控制机械臂按不同方向移动：
  
  - 以机械臂末端坐标系为参考，机械臂末端运动轨迹需尽可能覆盖所有的运动自由度（x, y, z, roll, pitch, yaw）
  
  - 每次运动完成后，保持静止不低于4s

- 参考资源:
  - [示例视频](https://cdn.robosense.cn/AC_wiki/sensor_to_mechanical_arm.mp4)
  - [demo数据](https://cdn.robosense.cn/AC_wiki/sensor_to_arm_calib.zip)
