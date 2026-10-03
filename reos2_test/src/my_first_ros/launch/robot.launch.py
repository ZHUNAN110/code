# robot.launch.py —— 一键启动 reos2_test 下 my_first_ros 的全部节点
#
# 用法：
#   ros2 launch my_first_ros robot.launch.py
#   ros2 launch my_first_ros robot.launch.py max_linear:=0.8 friction_coeff:=0.3
#
# 启动的节点：
#   - robot_node：机器人速度指令处理 + 状态处理（障碍物/驱动器/地面摩擦安全）
#   - sub_node  ：教程节点，订阅 /cmd_vel 打印速度指令
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _float_arg(name: str, default: float):
    """声明一个数值型 launch 参数，返回 (声明, float 表达式)。"""
    decl = DeclareLaunchArgument(name, default_value=str(default))
    expr = PythonExpression(["float('", LaunchConfiguration(name), "')"])
    return decl, expr


def generate_launch_description():
    ld = []
    params = {}

    # —— 关键可调参数（可在命令行覆盖） ——
    for name, default in [
        ("max_linear", 0.50),        # 最大线速度 (m/s)
        ("max_angular", 1.00),       # 最大角速度 (rad/s)
        ("max_linear_accel", 1.00),  # 最大线加速度 (m/s^2)
        ("max_angular_accel", 2.00), # 最大角加速度 (rad/s^2)
        ("cg_height", 0.15),         # 重心离地高度 (m)
        ("friction_coeff", 0.60),    # 地面摩擦系数 μ（0~1）
    ]:
        decl, expr = _float_arg(name, default)
        ld.append(decl)
        params[name] = expr

    # —— 其余参数直接给默认值 ——
    params.update({
        # 几何 / 机械
        "wheel_radius": 0.05,
        "wheel_base": 0.20,
        "battery_drain": 0.02,
        "max_wheel_speed": 20.0,
        "control_rate": 50.0,
        "rollover_safety_factor": 0.50,
        # 障碍物
        "stop_distance": 0.15,
        "slow_distance": 0.50,
        "emergency_brake_distance": 0.30,
        "collision_distance": 0.10,
        "turn_direction": 1,
        "vehicle_width": 0.30,
        "clearance_margin": 0.10,
        "gap_pass_speed": 0.20,
        # 驱动器异常
        "undervoltage_warn": 22.0,
        "undervoltage_critical": 20.0,
        "overtemp_warn": 60.0,
        "overtemp_critical": 80.0,
        # 地面摩擦
        "traction_margin": 0.80,
        "braking_distance": 0.15,
        # 看门狗
        "cmd_timeout": 0.5,
    })

    robot_node = Node(
        package="my_first_ros",
        executable="robot_node",
        name="robot_node",
        output="screen",
        parameters=[params],
    )

    # 教程节点：订阅 /cmd_vel 打印速度指令
    sub_node = Node(
        package="my_first_ros",
        executable="sub_node",
        name="sub_node",
        output="screen",
    )

    ld.append(robot_node)
    ld.append(sub_node)
    return LaunchDescription(ld)
