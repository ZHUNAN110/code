cmake_minimum_required(VERSION 3.8)
project(my_custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 把 msg/ 下的接口定义生成 C++ 与 Python 代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "msg/RobotState.msg"
  "msg/Obstacle.msg"
  "msg/DriveStatus.msg"
  "msg/GroundCondition.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
