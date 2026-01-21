#include "rdk_imu_data_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <memory>

std::shared_ptr<ImuDataNode> g_imu_node = nullptr;

void signalHandler(int signum) {
    if (g_imu_node) {
        RCLCPP_INFO(g_imu_node->get_logger(), 
                   "收到信号 %d，正在关闭rdk_imu_node节点...", 
                   signum);
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "启动rdk_imu_node节点");
    
    try{
        // 创建节点选项
        rclcpp::NodeOptions options;
        
        // 创建IMU节点
        g_imu_node = std::make_shared<ImuDataNode>("rdk_imu_node", options);
        
        // 注册信号处理
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        RCLCPP_INFO(g_imu_node->get_logger(), "IMU节点开始运行");
        
        // 运行节点
        rclcpp::spin(g_imu_node);
        
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(rclcpp::get_logger("main"), 
                    "IMU节点启动失败: %s", 
                    e.what());
        return 1;
    }
    
    // 清理
    g_imu_node.reset();
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "rdk_imu_node节点已关闭");
    return 0;
}