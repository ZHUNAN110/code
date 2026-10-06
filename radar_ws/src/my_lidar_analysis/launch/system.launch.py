# 一键启动分析系统，可用参数切换「假数据 / 真雷达」
# 用假数据（默认）：ros2 launch my_lidar_analysis system.launch.py
# 用真雷达：         ros2 launch my_lidar_analysis system.launch.py use_fake_data:=false
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # —— 参数：use_fake_data（true=假数据，false=真雷达）——
    use_fake_data = LaunchConfiguration('use_fake_data')
    declare_use_fake_data = DeclareLaunchArgument(
        'use_fake_data',
        default_value='true',
        description='true=用假数据测试；false=用真实雷达(livox+fast_lio)',
    )

    # —— 节点3：假数据发布器（只在 use_fake_data=true 时启动）——
    fake_pub = Node(
        package='my_lidar_analysis',
        executable='fake_data_publisher',
        name='fake_data_publisher',
        output='screen',
        condition=IfCondition(use_fake_data),
    )

    # —— 真雷达：livox 驱动（只在 use_fake_data=false 时启动）——
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 4,        # 4 = 同时发 PointCloud2 + CustomMsg + IMU
            'multi_topic': 0,        # 0 = 所有雷达共用一个话题
            'data_src': 0,           # 0 = 真实雷达
            'publish_freq': 10.0,    # 点云发布频率
            'frame_id': 'livox_frame',
            # 雷达连接配置：写死了雷达 IP 192.168.1.130、本机 IP 192.168.1.41
            'user_config_path': os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'config', 'MID360_config.json'),
        }],
        condition=UnlessCondition(use_fake_data),
    )

    # —— 真雷达：fast_lio（只在 use_fake_data=false 时启动）——
    # 复用 fast_lio 包自带的 mapping.launch.py，关掉 rviz，用 mid360 配置
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('fast_lio'), '/launch/mapping.launch.py']
        ),
        launch_arguments=[
            ('config_file', 'mid360.yaml'),
            ('rviz', 'false'),
        ],
        condition=UnlessCondition(use_fake_data),
    )

    # —— 节点1 + 节点2：采集统计 + 展示（两种模式都要启动）——
    analyzer = Node(
        package='my_lidar_analysis',
        executable='analyzer_node',
        name='analyzer_node',
        output='screen',
    )

    display = Node(
        package='my_lidar_analysis',
        executable='display_node',
        name='display_node',
        output='screen',
    )

    return LaunchDescription([
        declare_use_fake_data,
        fake_pub,
        livox_driver,
        fast_lio_launch,
        analyzer,
        display,
    ])
