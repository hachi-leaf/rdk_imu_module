#ifndef __RDK_IMU_DATA_NODE_HPP_
#define __RDK_IMU_DATA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

extern "C" {
#include "rdk_imu_module.h"
}

class ImuDataNode:public rclcpp::Node{
    public:
        explicit ImuDataNode(
            const std::string& node_name,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~ImuDataNode();

    private:
        void initParameters();
        void initPublisher();
        void initTimer();

        // 成员变量
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::TimerBase::SharedPtr public_timer_;
        struct imu_state imu_st;
        struct imu_data imu_dt;
        uint64_t publish_count_;

        // 参数变量
        std::string topic_name_;
        float publish_rate_;
        int acc_range_;
        int acc_bwp_;
        int acc_odr_;
        int gyro_range_;
        int gyro_bandwidth_;
        float g_value_;

        // 回调函数
        void timerCallback();
};

#endif
