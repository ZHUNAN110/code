cmake_minimum_required(VERSION 3.8)
project(my_first_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_custom_msgs REQUIRED)

# 2. 定义可执行文件
add_executable(pub_node src/pub_node.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs)

add_executable(sub_node src/sub_node.cpp)
ament_target_dependencies(sub_node rclcpp geometry_msgs)

add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node rclcpp geometry_msgs my_custom_msgs)

# 3. 安装可执行文件：必须装到 lib/${PROJECT_NAME}，ros2 run 才找得到
install(TARGETS
  pub_node
  sub_node
  robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 安装 launch 文件：装到 share/${PROJECT_NAME}，ros2 launch 才找得到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
