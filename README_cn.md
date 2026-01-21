[English](./README.MD) | 简体中文 <br>
<br>

# **RDK IMU Module Package**

## ———— *RDK IMU Module驱动包 for RDK X5* ————

## **一、功能介绍**

`RDK IMU Module Package`是一个用于驱动RDK IMU Module获得IMU的加速度计与陀螺仪数据的使用ROS2/TROS功能包。<br>

`RDK IMU Module Package`基于Linux的I2C/SPI通用驱动点亮sensor，具体实现细节参考[RDK_IMU_Module_us](https://github.com/hachi-leaf/RDK_IMU_Module_us)。<br>

`RDK IMU Module Package`的ROS2包名为`rdk_imu_module`，包含一个节点名为`rdk_imu_node`，支持RDK IMU Module设备SPI/I2C总线与地址自动扫描。<br>

## **二、环境准备与编译**

1. 硬件准备RDK X5开发板，RDK IMU Module，并正确安装、上电；<br>

2. RDK X5具备正常RDK OS系统，正确安装TROS，并正确设置TROS环境变量：`source /opt/tros/humble/setup.bash`，正确安装ROS2编译工具`colcon`。<br>

3. 将`rdk_imu_module`目录下载到工作空间目录下的src目录下，在工作空间下键入`colcon build --packages-select rdk_imu_module`，终端输出`Finished <<< rdk_imu_module [x.xs]`信息代表编译完成。<br>

4. 在工作空间下键入`source ./install/setup.sh`以将编译完成的包被添加到环境变量中。<br>

## **三、节点运行**

`RDK IMU Module Package`只有一个节点`rdk_imu_node`，键入`ros2 run rdk_imu_module rdk_imu_node`以运行，正常输出以下内容：<br>

```text
root@ubuntu:~/imu_ros2_ws# ros2 run rdk_imu_module rdk_imu_node
[INFO] [1768992309.819989210] [main]: 启动rdk_imu_node节点
[INFO] [1768992310.196471269] [rdk_imu_node]: 参数配置完成 | 频率: 200.0 Hz | 话题: rdk_imu_data | 加速度计[量程:3 滤波:2 ODR:12] | 陀螺仪[量程:0 ODR与带宽:0]
[INFO] [1768992310.202321105] [rdk_imu_node]: 发布者初始化成功 | 话题: rdk_imu_data
[INFO] [1768992310.202555355] [rdk_imu_node]: 定时器初始化成功 | 周期: 5 ms
[INFO] [1768992310.411805017] [rdk_imu_node]: IMU设备扫描成功
[INFO] [1768992310.568062785] [rdk_imu_node]: ImuDataNode初始化成功 | 节点名: rdk_imu_node | 发布频率: 200.0 Hz | 话题: rdk_imu_data
[INFO] [1768992310.568365451] [rdk_imu_node]: IMU节点开始运行
```

新建一个终端，键入`ros2 topic list`可以看到`/rdk_imu_data`话题被建立。<br>

键入`ros2 topic echo /rdk_imu_data`循环打印IMU数据，可见rdk_imu_data话题的数据是标准`sensor_msgs::msg::Imu`类型接口：<br>

```text
header:
  stamp:
    sec: 1768992532
    nanosec: 438521088
  frame_id: ''
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.07456851052221866
  y: 0.029827404208887465
  z: 0.07030745277809189
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.19379882514476776
  y: 0.26557618379592896
  z: -9.646875381469727
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

该话题中，时间戳数据与系统时间戳对齐且与加速度数据强相关，方向数据不可用，角速度单位为`rad/s`，线加速度单位为`m/s^2`。<br>

## **四、参数文件**

默认参数文件位于`./config/rdk_imu_node_default_params.yaml`，`acc_bwp`、`acc_range`和`acc_odr`为加速度计参数，`gyro_bandwidth`和`gyro_range`为陀螺仪参数，改变`g_value`可改变加速度计三轴输出数据的缩放系数，改变`publish_rate`可以改变轮询频率(Hz)。<br>

## **五、版本信息**

- 项目发布时间：2026-1-21

- 项目版本：v1.0.0

- 版本时间：2026-1-21

- 项目最后更改时间：2026-1-21

- README最后更改时间：2026-1-20
