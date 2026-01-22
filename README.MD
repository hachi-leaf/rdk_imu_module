English | [简体中文](./README_cn.md) <br>
<br>

# **RDK IMU Module Package**

## ———— *RDK IMU Module Driver Package for RDK X3/X5* ————

## **1. Features**

`RDK IMU Module Package` is a ROS2/TROS package for driving the RDK IMU Module to obtain IMU accelerometer and gyroscope data.<br>

`RDK IMU Module Package` utilizes Linux's I2C/SPI general drivers to interface with the sensor. Implementation details can be found at [RDK_IMU_Module_us](https://github.com/hachi-leaf/RDK_IMU_Module_us).<br>

The ROS2 package name is `rdk_imu_module`, containing a node named `rdk_imu_node`. It supports automatic scanning of RDK IMU Module device SPI/I2C buses and addresses.<br>

## **2. Environment Setup and Compilation**

1. Hardware preparation: RDK X5 development board, RDK IMU Module, correctly installed and powered on.<br>

2. RDK X3/X5 should have a functional RDK OS system with TROS properly installed. Set up the TROS environment correctly: `source /opt/tros/humble/setup.bash`. Ensure the ROS2 build tool `colcon` is installed.<br>

3. Download the `rdk_imu_module` directory into the `src` folder of your workspace. Navigate to the workspace directory and run `colcon build --packages-select rdk_imu_module`. The terminal output `Finished <<< rdk_imu_module [x.xs]` indicates successful compilation.<br>

4. In the workspace, run `source ./install/setup.sh` to add the compiled package to the environment variables.<br>

## **3. Node Execution**

`RDK IMU Module Package` contains only one node: `rdk_imu_node`. Run it by typing `ros2 run rdk_imu_module rdk_imu_node`. If normal will output is as follows:<br>

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

Open a new terminal and enter`ros2 topic list`to see that the`/rdk_imu_data`topic has been created.<br>

Enter`ros2 topic echo /rdk_imu_data`to cyclically print IMU data. The data on the rdk_imu_data topic adheres to the standard`sensor_msgs::msg::Imu`type interface:<br>

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

In this topic, the timestamp data is aligned with the system timestamp and is strongly correlated with acceleration data. Orientation data is unavailable. Angular velocity unit is `rad/s`, and linear acceleration unit is`m/s^2`。<br>

## **4. Parameter Files**

The default parameter file is located at`./config/rdk_imu_node_default_params.yaml`.`acc_bwp`,`acc_range`and`acc_odr`are accelerometer parameters; `gyro_bandwidth`and`gyro_range`are gyroscope parameters. Changing`g_value`adjusts the scaling factor for accelerometer three-axis output data. Changing`publish_rate`adjusts the polling frequency (Hz).<br>

## **5. Version Information**

- Project Release Date: January 21, 2026

- Project Version: v1.0.0

- Version Date: January 21, 2026

- Last Modified: January 21, 2026

- README Last Modified: January 22, 2026
