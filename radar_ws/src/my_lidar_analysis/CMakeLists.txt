cmake_minimum_required(VERSION 3.8)
project(my_lidar_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # 生成自定义消息用

# 生成自定义消息（把 msg/AnalysisResult.msg 变成 C++ 头文件）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnalysisResult.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# —— 节点1：数据采集 + 统计，发布结果 ——
add_executable(analyzer_node src/analyzer_node.cpp)
ament_target_dependencies(analyzer_node rclcpp sensor_msgs nav_msgs tf2_msgs)
rosidl_target_interfaces(analyzer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# —— 节点2：订阅结果 + 打印 ——
add_executable(display_node src/display_node.cpp)
ament_target_dependencies(display_node rclcpp)
rosidl_target_interfaces(display_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# —— 节点3：假数据发布器（测试用，不需要自定义消息）——
add_executable(fake_data_publisher src/fake_data_publisher.cpp)
ament_target_dependencies(fake_data_publisher rclcpp sensor_msgs nav_msgs tf2_msgs)

# 安装，让 ros2 run 能直接找到它们
install(TARGETS analyzer_node display_node fake_data_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
