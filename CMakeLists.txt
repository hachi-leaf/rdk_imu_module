cmake_minimum_required(VERSION 3.8)
project(rdk_imu_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加可执行文件
add_executable(rdk_imu_node
  src/main.cpp
  src/rdk_imu_data_node.cpp
  src/rdk_imu_module_data.c
)

# 使用ament_target_dependencies正确配置依赖
ament_target_dependencies(rdk_imu_node
  rclcpp
  sensor_msgs
)

target_include_directories(rdk_imu_node PRIVATE 
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rdk_imu_module
)

# 设置C标准为C11，C++标准为C++17
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项
target_compile_options(rdk_imu_node PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# 链接库 - 使用ROS2提供的现代CMake目标
target_link_libraries(rdk_imu_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

# 安装目标
install(TARGETS rdk_imu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
)

ament_export_include_directories(include)
ament_export_libraries(rdk_imu_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()