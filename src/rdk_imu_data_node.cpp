#include "rclcpp/rclcpp.hpp"
#include "rdk_imu_data_node.hpp"

ImuDataNode::ImuDataNode(
    const std::string& node_name,
    const rclcpp::NodeOptions& options)
    : Node(node_name, options),
      imu_st(RDK_IMU_Get_Initial_State()),
      imu_dt(RDK_IMU_Get_Initial_Data())
{
    enum rdk_imu_error imu_error = RDK_IMU_OK;
    int cumulative_error = 0;

    this->initParameters();
    this->initPublisher();
    this->initTimer();

    // 扫描IMU设备
    imu_error = RDK_IMU_All_Device_Scan(&this->imu_st);
    cumulative_error |= imu_error;
    if (imu_error != RDK_IMU_OK) {
        RCLCPP_ERROR(this->get_logger(), "未发现IMU设备");
    } else {
        RCLCPP_INFO(this->get_logger(), "IMU设备扫描成功");
    }

    // IMU软复位
    imu_error = RDK_IMU_Accel_Reset(&this->imu_st);
    RCLCPP_DEBUG(this->get_logger(), "加速度计复位返回值: %d", static_cast<int>(imu_error));
    
    imu_error = RDK_IMU_Gyro_Reset(&this->imu_st);
    RCLCPP_DEBUG(this->get_logger(), "陀螺仪复位返回值: %d", static_cast<int>(imu_error));

    // IMU上电
    imu_error = RDK_IMU_Accel_Pwr_Set(&this->imu_st, ACC_PWR_ON);
    cumulative_error |= imu_error;
    RCLCPP_DEBUG(this->get_logger(), "加速度计上电返回值: %d", static_cast<int>(imu_error));
    
    imu_error = RDK_IMU_Gyro_Pwr_Set(&this->imu_st, GYRO_LPM1_NORMAL);
    cumulative_error |= imu_error;
    RCLCPP_DEBUG(this->get_logger(), "陀螺仪上电返回值: %d", static_cast<int>(imu_error));

    // IMU参数配置
    imu_error = RDK_IMU_Accel_Config(&this->imu_st,
        static_cast<enum bmi088_acc_range>(this->acc_range_),
        static_cast<enum bmi088_acc_bwp>(this->acc_bwp_),
        static_cast<enum bmi088_acc_odr>(this->acc_odr_));
    cumulative_error |= imu_error;
    RCLCPP_DEBUG(this->get_logger(), "加速度计配置返回值: %d", static_cast<int>(imu_error));
    
    imu_error = RDK_IMU_Gyro_Config(&this->imu_st,
        static_cast<enum bmi088_gyro_range>(this->gyro_range_),
        static_cast<enum bmi088_gyro_bandwidth>(this->gyro_bandwidth_));
    cumulative_error |= imu_error;
    RCLCPP_DEBUG(this->get_logger(), "陀螺仪配置返回值: %d", static_cast<int>(imu_error));

    // 初始化结果检查
    if (cumulative_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "IMU初始化失败，错误值掩码: 0x%X", cumulative_error);
        throw std::runtime_error("IMU初始化失败");
    } else {
        RCLCPP_INFO(this->get_logger(), 
                   "ImuDataNode初始化成功 | 节点名: %s | 发布频率: %.1f Hz | 话题: %s",
                   node_name.c_str(), publish_rate_, topic_name_.c_str());
    }
}


ImuDataNode::~ImuDataNode() {
    RCLCPP_INFO(this->get_logger(), "ImuDataNode析构");
}

void ImuDataNode::initParameters() {
    // 声明并描述节点参数
    this->declare_parameter<std::string>("topic_name", "rdk_imu_data",
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("IMU数据发布话题名称"));
        
    this->declare_parameter<float>("publish_rate", 200.0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("数据发布频率(Hz)"));
        
    this->declare_parameter<int>("acc_range", 3,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("加速度计量程 (0:±3g, 1:±6g, 2:±12g, 3:±24g)"));
        
    this->declare_parameter<int>("acc_bwp", 2,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("加速度计滤波器配置"));
        
    this->declare_parameter<int>("acc_odr", 12,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("加速度计输出数据速率"));
        
    this->declare_parameter<int>("gyro_range", 0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("陀螺仪量程 (0:±125°/s, 1:±250°/s, 2:±500°/s, 3:±1000°/s, 4:±2000°/s)"));
        
    this->declare_parameter<int>("gyro_bandwidth", 0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("陀螺仪带宽配置"));

    this->declare_parameter<float>("g_value", 9.8f,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("重力加速度值(m/s^2)"));

    // 获取参数值
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("acc_range", acc_range_);
    this->get_parameter("acc_bwp", acc_bwp_);
    this->get_parameter("acc_odr", acc_odr_);
    this->get_parameter("gyro_range", gyro_range_);
    this->get_parameter("gyro_bandwidth", gyro_bandwidth_);
    this->get_parameter("g_value", g_value_);

    // 参数有效性检查
    if (publish_rate_ <= 0.0f || publish_rate_ > 1000.0f) {
        RCLCPP_WARN(this->get_logger(), 
                   "无效的发布频率 %.2f Hz，使用默认值 100.0 Hz", 
                   publish_rate_);
        publish_rate_ = 100.0f;
    }

    // 记录参数配置
    RCLCPP_INFO(this->get_logger(), 
               "参数配置完成 | 频率: %.1f Hz | 话题: %s | "
               "加速度计[量程:%d 滤波:%d ODR:%d] | "
               "陀螺仪[量程:%d ODR与带宽:%d]",
               publish_rate_, 
               topic_name_.c_str(), 
               acc_range_, acc_bwp_, acc_odr_, 
               gyro_range_, gyro_bandwidth_);
}

void ImuDataNode::initPublisher() {
    try{
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            topic_name_, rclcpp::QoS(10));
        
        RCLCPP_INFO(this->get_logger(), "发布者初始化成功 | 话题: %s", topic_name_.c_str());
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "发布者初始化失败: %s", e.what());
        throw; 
    }
}

void ImuDataNode::initTimer() {
    try {
        auto timer_period = std::chrono::milliseconds(
            static_cast<int64_t>(1000.0 / publish_rate_));

        // 设置回调 
        this->public_timer_ = this->create_wall_timer(timer_period,
            std::bind(&ImuDataNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "定时器初始化成功 | 周期: %ld ms", timer_period.count());
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "定时器初始化失败: %s", e.what());
        throw;
    }
}

// 定时器回调函数
void ImuDataNode::timerCallback() {
    try{
        // 创建imu数据
        auto imu_msg = sensor_msgs::msg::Imu();

        // 硬件数据读取
        RDK_IMU_Read(&this->imu_st, &this->imu_dt);

        // 设置消息头
        imu_msg.header.stamp = rclcpp::Time(this->imu_dt.accel.timestamp*1e3);  // 使用加速度计的时间戳

        // accel
        imu_msg.linear_acceleration.x = this->imu_dt.accel.x * this->g_value_;
        imu_msg.linear_acceleration.y = this->imu_dt.accel.y * this->g_value_;
        imu_msg.linear_acceleration.z = this->imu_dt.accel.z * this->g_value_;

        // angvel
        imu_msg.angular_velocity.x = this->imu_dt.angvel.x * M_PI / 180.0f;
        imu_msg.angular_velocity.y = this->imu_dt.angvel.y * M_PI / 180.0f;
        imu_msg.angular_velocity.z = this->imu_dt.angvel.z * M_PI / 180.0f;

        // 方向数据不可用
        imu_msg.orientation_covariance[0] = -1.0;

        // 发布消息
        imu_publisher_->publish(imu_msg);

        // 调试输出（每100条输出一次）
        if(publish_count_++ % 100 == 0){
            RCLCPP_DEBUG(this->get_logger(), 
                        "已发布 %ld 条IMU消息", 
                        publish_count_);
            
            // 打印当前IMU数据
            RCLCPP_DEBUG(this->get_logger(),
                        "IMU数据 - timestamp: %d.%09d"
                        "角速度: [%.3f, %.3f, %.3f], "
                        "加速度: [%.3f, %.3f, %.3f]",
                        imu_msg.header.stamp.sec,
                        imu_msg.header.stamp.nanosec,
                        imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z,
                        imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "定时器回调错误: %s", e.what());
    }
}
