// robot_node.cpp —— 简单差分驱动机器人：速度指令处理 + 状态处理
//
// 功能：
//   1. 速度指令处理：订阅 /cmd_vel（Twist），限幅后作为"原始目标速度"；
//      用 KeepLast(1)+BestEffort 队列避免积压导致执行过时指令；
//   2. 安全速度（防打滑 / 防翻车）：
//        · 加速度限制：实际速度以 max_linear_accel/max_angular_accel 斜坡逼近目标；
//        · 侧向离心限制：|vx*wz| 超过由重心高反推的阈值时压低角速度，防转弯侧翻；
//        · 地面摩擦自适应：光滑无纹理地面(低 μ)时降低最高速、加速度与侧向限制，防打滑/甩尾；
//        · 机械极限：轮速钳制在 max_wheel_speed 以内；
//   3. 障碍物处理：按类型区分应对——
//        · 台阶(小)：直接跳跃越过；大型刚体实心→减速/停车，中间有空洞→按车宽判断通过或绕开；
//        · 行人：即将相撞时紧急制动并报警，来不及制动则紧急转向绕开；
//   4. 驱动器异常处理（订阅 /drive_status，每周期持续生效）：
//        · 低压 / 过热：告警区减速运行，临界区硬停车并报警；
//        · 堵转 / 驱动器故障：立即硬停车并报警（保护电机与驱动器）；
//   5. 指令看门狗：超时未收到新指令自动停车；
//   6. 状态处理：周期发布 /robot_state（含故障码）。
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_custom_msgs/msg/robot_state.hpp"
#include "my_custom_msgs/msg/obstacle.hpp"
#include "my_custom_msgs/msg/drive_status.hpp"
#include "my_custom_msgs/msg/ground_condition.hpp"

using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node
{
public:
  RobotNode()
  : Node("robot_node")
  {
    // —— 参数：机器人几何尺寸与性能限制 ——
    this->declare_parameter<double>("wheel_radius", 0.05);   // 轮子半径 (m)
    this->declare_parameter<double>("wheel_base", 0.20);      // 左右轮间距 (m)
    this->declare_parameter<double>("max_linear", 0.50);      // 最大线速度 (m/s)
    this->declare_parameter<double>("max_angular", 1.00);     // 最大角速度 (rad/s)
    this->declare_parameter<double>("battery_drain", 0.02);   // 运动时每秒耗电 (%)
    // —— 安全速度：防打滑 / 防翻车 ——
    this->declare_parameter<double>("max_linear_accel", 1.00);   // 最大线加速度 (m/s^2)，防打滑/翘头
    this->declare_parameter<double>("max_angular_accel", 2.00);  // 最大角加速度 (rad/s^2)
    this->declare_parameter<double>("cg_height", 0.15);             // 重心离地高度 (m)
    this->declare_parameter<double>("rollover_safety_factor", 0.50); // 防侧翻安全系数（<1，越小越保守）
    this->declare_parameter<double>("max_wheel_speed", 20.0);    // 轮子机械极限转速 (rad/s)
    this->declare_parameter<double>("control_rate", 50.0);       // 速度控制环频率 (Hz)
    // 障碍物距离阈值
    this->declare_parameter<double>("stop_distance", 0.15);            // 大型障碍停车距离 (m)
    this->declare_parameter<double>("slow_distance", 0.50);            // 开始减速距离 (m)
    this->declare_parameter<double>("emergency_brake_distance", 0.30); // 行人紧急制动距离 (m)
    this->declare_parameter<double>("collision_distance", 0.10);       // 行人来不及制动→转向 (m)
    this->declare_parameter<int>("turn_direction", 1);                 // 转向方向 +1左转 -1右转
    this->declare_parameter<double>("vehicle_width", 0.30);            // 自身车辆宽度 (m)
    this->declare_parameter<double>("clearance_margin", 0.10);         // 通过空洞所需的总余量 (m)
    this->declare_parameter<double>("gap_pass_speed", 0.20);           // 穿过空洞时的限速 (m/s)
    // —— 驱动器异常阈值 ——
    this->declare_parameter<double>("undervoltage_warn", 22.0);      // 低压告警阈值 (V)，低于则减速运行
    this->declare_parameter<double>("undervoltage_critical", 20.0);  // 低压临界阈值 (V)，低于则硬停车
    this->declare_parameter<double>("overtemp_warn", 60.0);          // 过热告警阈值 (℃)，高于则减速运行
    this->declare_parameter<double>("overtemp_critical", 80.0);      // 过热临界阈值 (℃)，高于则硬停车
    // —— 地面摩擦自适应 ——
    this->declare_parameter<double>("friction_coeff", 0.60);       // 地面摩擦系数 μ 默认值（无读数时用）
    this->declare_parameter<double>("traction_margin", 0.80);      // 附着安全系数（<1，越小越保守）
    this->declare_parameter<double>("braking_distance", 0.15);     // 安全制动距离 (m)，用于反推最高速
    this->declare_parameter<double>("cmd_timeout", 0.5);             // 指令超时时间 (s)
    wheel_radius_  = this->get_parameter("wheel_radius").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    max_linear_    = this->get_parameter("max_linear").as_double();
    max_angular_   = this->get_parameter("max_angular").as_double();
    battery_drain_ = this->get_parameter("battery_drain").as_double();
    max_linear_accel_  = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    cg_height_ = this->get_parameter("cg_height").as_double();
    rollover_safety_factor_ = this->get_parameter("rollover_safety_factor").as_double();
    // 反推防侧翻阈值：a_tip = g * (轮距/2) / 重心高，再乘安全系数
    max_lateral_accel_ = rollover_safety_factor_ * 9.81 * (wheel_base_ / 2.0) / cg_height_;
    max_wheel_speed_   = this->get_parameter("max_wheel_speed").as_double();
    control_rate_      = this->get_parameter("control_rate").as_double();
    stop_distance_ = this->get_parameter("stop_distance").as_double();
    slow_distance_ = this->get_parameter("slow_distance").as_double();
    emergency_brake_distance_ = this->get_parameter("emergency_brake_distance").as_double();
    collision_distance_ = this->get_parameter("collision_distance").as_double();
    turn_direction_ = this->get_parameter("turn_direction").as_int();
    vehicle_width_ = this->get_parameter("vehicle_width").as_double();
    clearance_margin_ = this->get_parameter("clearance_margin").as_double();
    gap_pass_speed_ = this->get_parameter("gap_pass_speed").as_double();
    undervoltage_warn_ = this->get_parameter("undervoltage_warn").as_double();
    undervoltage_critical_ = this->get_parameter("undervoltage_critical").as_double();
    overtemp_warn_ = this->get_parameter("overtemp_warn").as_double();
    overtemp_critical_ = this->get_parameter("overtemp_critical").as_double();
    friction_coeff_ = this->get_parameter("friction_coeff").as_double();
    traction_margin_ = this->get_parameter("traction_margin").as_double();
    braking_distance_ = this->get_parameter("braking_distance").as_double();
    cmd_timeout_   = this->get_parameter("cmd_timeout").as_double();

    // —— 1. 速度指令处理：订阅 /cmd_vel ——
    // KeepLast(1) + BestEffort：队列只保留最新一条，积压的旧指令直接丢弃
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });

    // —— 2. 障碍物处理：订阅 /obstacle ——
    obstacle_sub_ = this->create_subscription<my_custom_msgs::msg::Obstacle>(
      "obstacle", 10,
      [this](const my_custom_msgs::msg::Obstacle::SharedPtr msg) { on_obstacle(msg); });

    // —— 3. 驱动器异常处理：订阅 /drive_status ——
    drive_status_sub_ = this->create_subscription<my_custom_msgs::msg::DriveStatus>(
      "drive_status", 10,
      [this](const my_custom_msgs::msg::DriveStatus::SharedPtr msg) { on_drive_status(msg); });

    // —— 3.5 地面摩擦自适应：订阅 /ground_condition ——
    ground_condition_sub_ = this->create_subscription<my_custom_msgs::msg::GroundCondition>(
      "ground_condition", 10,
      [this](const my_custom_msgs::msg::GroundCondition::SharedPtr msg) { on_ground_condition(msg); });

    // —— 4. 状态处理：周期发布 /robot_state ——
    state_pub_ = this->create_publisher<my_custom_msgs::msg::RobotState>("robot_state", 10);
    state_timer_ = this->create_wall_timer(200ms, [this]() { publish_state(); });

    // 速度控制环：每周期重评估障碍物/驱动器约束，再以加速度上限逼近目标
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(control_period, [this]() { update_velocity(); });

    // 电池模拟：每秒扣电
    battery_timer_ = this->create_wall_timer(1s, [this]() { drain_battery(); });

    // 指令看门狗：超过 cmd_timeout 未收到新指令就停车
    watchdog_timer_ = this->create_wall_timer(100ms, [this]() { check_cmd_stale(); });

    RCLCPP_INFO(this->get_logger(),
      "robot_node 已启动：订阅 /cmd_vel、/obstacle、/drive_status、/ground_condition，发布 /robot_state"
      "（限速 %.2f m/s / %.2f rad/s，限加速 %.2f m/s^2 / %.2f rad/s^2，"
      "重心高 %.2f m × 安全系数 %.2f → 防侧翻离心 %.2f m/s^2，"
      "地面 μ=%.2f × 附着系数 %.2f、制动距离 %.2f m，"
      "低压 %.0f/%.0f V，过热 %.0f/%.0f ℃，控制率 %.0f Hz，指令超时 %.2f s）",
      max_linear_, max_angular_, max_linear_accel_, max_angular_accel_,
      cg_height_, rollover_safety_factor_, max_lateral_accel_,
      friction_coeff_, traction_margin_, braking_distance_,
      undervoltage_critical_, undervoltage_warn_, overtemp_warn_, overtemp_critical_,
      control_rate_, cmd_timeout_);
  }

private:
  // 速度指令处理：限幅后作为"原始目标速度"（障碍物/驱动器约束在控制环里叠加）
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();   // 记录收到指令的时间，供看门狗判断
    has_cmd_ = true;

    raw_target_vx_ = clamp(msg->linear.x, -max_linear_, max_linear_);
    raw_target_wz_ = clamp(msg->angular.z, -max_angular_, max_angular_);

    RCLCPP_INFO(this->get_logger(), "原始目标速度 vx=%.2f m/s, wz=%.2f rad/s",
      raw_target_vx_, raw_target_wz_);
  }

  // 速度控制环：地面摩擦 → 障碍物/驱动器约束 → 加速度斜坡 → 防侧翻 → 逆解算
  void update_velocity()
  {
    // —— 0. 地面摩擦自适应：按 μ 反推速度/加速度上限 ——
    const double mu = friction_coeff_;
    const double mu_g = mu * 9.81 * traction_margin_;   // 附着极限加速度 μ·g·margin
    // 加速不打滑：a ≤ μ·g·margin
    const double a_lin_eff = std::min(max_linear_accel_, mu_g);
    // 转向不打滑：α·(轮距/2) ≤ μ·g·margin
    const double a_ang_eff = std::min(max_angular_accel_, mu_g / (wheel_base_ / 2.0));
    // 侧向不打滑：取翻车阈值与 μ·g 的较小者
    const double a_lat_eff = std::min(max_lateral_accel_, mu_g);
    // 制动距离不变：v ≤ sqrt(2·μ·g·d_brake)，光滑地面降最高速
    const double v_max_eff = std::min(max_linear_, std::sqrt(2.0 * mu * 9.81 * braking_distance_));

    if (mu < 0.3) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "地面光滑(μ=%.2f)，限制最高速 %.2f m/s、加速 %.2f m/s^2、侧向 %.2f m/s^2",
        mu, v_max_eff, a_lin_eff, a_lat_eff);
    }

    double vx = clamp(raw_target_vx_, -v_max_eff, v_max_eff);
    double wz = raw_target_wz_;

    apply_obstacle_safety(vx, wz);   // 障碍物应对（修改目标）
    apply_drive_safety(vx, wz);      // 驱动器异常应对（可能触发硬停车）

    if (hard_stop_) {
      // 硬故障：立即切断，不再按加速度斜坡
      vx_ = 0.0;
      wz_ = 0.0;
    } else {
      const double dt = 1.0 / control_rate_;
      // 1. 限加速度：速度只能以 a_*_eff 变化，避免突变打滑/翘头
      vx_ = rate_limit(vx, vx_, a_lin_eff * dt);
      wz_ = rate_limit(wz, wz_, a_ang_eff * dt);

      // 2. 限侧向离心加速度：|vx*wz| 过大时压低角速度，防侧翻/侧滑
      if (std::abs(vx_) > 1e-3) {
        const double max_wz = a_lat_eff / std::abs(vx_);
        if (std::abs(wz_) > max_wz) {
          wz_ = std::copysign(max_wz, wz_);
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "线速度 %.2f m/s 时角速度被限制到 %.2f rad/s（防侧翻/侧滑）", vx_, wz_);
        }
      }
    }

    // 3. 机械极限：轮速钳制在 max_wheel_speed 以内
    // 逆解算：v = (wL + wR) * r / 2，w = (wR - wL) * r / L
    left_wheel_  = clamp((vx_ - wz_ * wheel_base_ / 2.0) / wheel_radius_,
                         -max_wheel_speed_, max_wheel_speed_);
    right_wheel_ = clamp((vx_ + wz_ * wheel_base_ / 2.0) / wheel_radius_,
                         -max_wheel_speed_, max_wheel_speed_);
  }

  // 障碍物回调：只记录最新一次读数
  void on_obstacle(const my_custom_msgs::msg::Obstacle::SharedPtr msg)
  {
    obstacle_type_ = msg->type;
    obstacle_distance_ = msg->distance;
    obstacle_height_ = msg->height;
    obstacle_valid_ = msg->valid;
    has_gap_ = msg->has_gap;
    gap_width_ = msg->gap_width;
  }

  // 驱动器状态回调：只记录最新一次读数
  void on_drive_status(const my_custom_msgs::msg::DriveStatus::SharedPtr msg)
  {
    voltage_ = msg->voltage;
    temperature_ = msg->temperature;
    stall_ = msg->stall;
    driver_fault_ = msg->driver_fault;
    drive_valid_ = msg->valid;
  }

  // 地面条件回调：更新摩擦系数
  void on_ground_condition(const my_custom_msgs::msg::GroundCondition::SharedPtr msg)
  {
    if (msg->valid) {
      friction_coeff_ = msg->friction_coeff;
    }
  }

  // 障碍物应对：按类型区分处理（修改的是"目标速度"）
  void apply_obstacle_safety(double & vx, double & wz)
  {
    if (!obstacle_valid_) {
      alarm_ = false;
      return;
    }

    const double d = obstacle_distance_;
    const double h = obstacle_height_;

    switch (obstacle_type_) {
      case OBSTACLE_STEP:
        // 台阶(小)：直接跳跃越过，不减速
        if (d <= slow_distance_ && vx > 0.0) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "前方 %.2f m 有 %.2f m 台阶，直接跳跃越过", d, h);
        }
        alarm_ = false;
        break;

      case OBSTACLE_LARGE:
        // 大型刚体障碍：先判断中间是否有可穿过的空洞
        if (has_gap_) {
          if (gap_width_ >= vehicle_width_ + clearance_margin_) {
            // 空洞够宽，减速通过
            if (vx > gap_pass_speed_) vx = gap_pass_speed_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "前方 %.2f m 刚体障碍有 %.2f m 空洞（车身 %.2f m + 余量 %.2f m），减速到 %.2f m/s 通过",
              d, gap_width_, vehicle_width_, clearance_margin_, vx);
            alarm_ = false;
          } else {
            // 空洞太窄，无法通过 → 绕开
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "前方 %.2f m 刚体障碍空洞 %.2f m 太窄（需 >= %.2f m），无法通过，绕开",
              d, gap_width_, vehicle_width_ + clearance_margin_);
            vx = 0.0;
            wz = max_angular_ * turn_direction_;
            alarm_ = false;
          }
          break;
        }
        // 实心刚体：执行原来的减速/停车
        if (d <= stop_distance_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "前方 %.2f m 有 %.2f m 大型障碍，已停车", d, h);
          vx = 0.0;
        } else if (d <= slow_distance_ && vx > 0.0) {
          const double scale = (d - stop_distance_) / (slow_distance_ - stop_distance_);
          vx *= scale;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "前方 %.2f m 有大型障碍，减速到 %.2f m/s", d, vx);
        }
        alarm_ = false;
        break;

      case OBSTACLE_PEDESTRIAN:
        if (d <= collision_distance_) {
          // 来不及制动：紧急转向绕开（优先不撞行人，哪怕撞到其他物体）
          alarm_ = true;
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "行人过近(%.2f m)已来不及制动，紧急转向绕开", d);
          vx = 0.0;
          wz = max_angular_ * turn_direction_;
        } else if (d <= emergency_brake_distance_) {
          // 紧急制动 + 报警
          alarm_ = true;
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "行人过近(%.2f m)，紧急制动并发出警报", d);
          vx = 0.0;
          wz = 0.0;
        } else if (d <= slow_distance_ && vx > 0.0) {
          // 行人尚远：先减速
          const double scale =
            (d - emergency_brake_distance_) / (slow_distance_ - emergency_brake_distance_);
          vx *= scale;
          alarm_ = false;
        } else {
          alarm_ = false;
        }
        break;

      default:
        alarm_ = false;
        break;
    }
  }

  // 驱动器异常应对：低压/过热→减速或硬停车，堵转/驱动器故障→硬停车
  void apply_drive_safety(double & vx, double & wz)
  {
    hard_stop_ = false;
    fault_ = FAULT_NONE;

    if (!drive_valid_) return;   // 无有效读数，视为健康

    // 堵转：电机锁死却仍被驱动，立即停车防止烧毁
    if (stall_) {
      fault_ |= FAULT_STALL;
      hard_stop_ = true;
      alarm_ = true;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "检测到堵转，立即硬停车保护电机");
      return;
    }

    // 驱动器故障（过流/过压/通讯异常等）：立即硬停车
    if (driver_fault_) {
      fault_ |= FAULT_DRIVER;
      hard_stop_ = true;
      alarm_ = true;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "驱动器故障，立即硬停车");
      return;
    }

    // 过热
    if (temperature_ >= overtemp_critical_) {
      fault_ |= FAULT_OVERTEMP;
      hard_stop_ = true;
      alarm_ = true;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "温度 %.1f ℃ 达到临界值，硬停车散热", temperature_);
      return;
    }

    // 低压临界
    if (voltage_ <= undervoltage_critical_) {
      fault_ |= FAULT_UNDERVOLTAGE;
      hard_stop_ = true;
      alarm_ = true;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "电压 %.1f V 低于临界值，硬停车保护电池", voltage_);
      return;
    }

    // 告警区：按剩余裕度线性降额（减速运行）
    double derate = 1.0;
    if (temperature_ >= overtemp_warn_) {
      fault_ |= FAULT_OVERTEMP;
      derate = std::min(derate,
        (overtemp_critical_ - temperature_) / (overtemp_critical_ - overtemp_warn_));
    }
    if (voltage_ <= undervoltage_warn_) {
      fault_ |= FAULT_UNDERVOLTAGE;
      derate = std::min(derate,
        (voltage_ - undervoltage_critical_) / (undervoltage_warn_ - undervoltage_critical_));
    }
    if (derate < 1.0) {
      vx *= derate;
      wz *= derate;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "电压 %.1f V / 温度 %.1f ℃，降额到 %.0f%% 运行", voltage_, temperature_, derate * 100.0);
    }
  }

  // 指令看门狗：长时间没收到新速度指令就把目标归零停车
  void check_cmd_stale()
  {
    if (!has_cmd_) return;

    const double dt = (this->now() - last_cmd_time_).seconds();
    if (dt > cmd_timeout_ && (raw_target_vx_ != 0.0 || raw_target_wz_ != 0.0)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "速度指令已 %.2f s 未更新，目标归零（实际速度按加速度上限减速）", dt);
      raw_target_vx_ = 0.0;
      raw_target_wz_ = 0.0;
    }
  }

  // 状态处理：把当前状态发布出去
  void publish_state()
  {
    my_custom_msgs::msg::RobotState state;
    state.vx = vx_;
    state.wz = wz_;
    state.left_wheel = left_wheel_;
    state.right_wheel = right_wheel_;
    state.battery = battery_;
    state.status = decide_status();
    state.alarm = alarm_;
    state.fault = fault_;
    state_pub_->publish(state);
  }

  void drain_battery()
  {
    if (!moving()) return;
    battery_ -= battery_drain_;
    if (battery_ < 0.0) battery_ = 0.0;
  }

  uint8_t decide_status() const
  {
    if (battery_ <= 20.0) return STATUS_LOW_BATTERY;
    return moving() ? STATUS_MOVING : STATUS_IDLE;
  }

  bool moving() const
  {
    constexpr double EPS = 1e-3;
    return std::abs(vx_) > EPS || std::abs(wz_) > EPS;
  }

  static double clamp(double v, double lo, double hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  // 斜坡逼近：目标相对当前的变化量不超过 max_delta
  static double rate_limit(double target, double current, double max_delta)
  {
    const double d = target - current;
    if (d > max_delta) return current + max_delta;
    if (d < -max_delta) return current - max_delta;
    return target;
  }

  enum Status : uint8_t { STATUS_IDLE = 0, STATUS_MOVING = 1, STATUS_LOW_BATTERY = 2 };
  enum ObstacleType : uint8_t { OBSTACLE_STEP = 0, OBSTACLE_LARGE = 1, OBSTACLE_PEDESTRIAN = 2 };
  enum Fault : uint8_t {
    FAULT_NONE = 0,
    FAULT_UNDERVOLTAGE = 1 << 0,
    FAULT_OVERTEMP = 1 << 1,
    FAULT_STALL = 1 << 2,
    FAULT_DRIVER = 1 << 3,
  };

  // 参数
  double wheel_radius_ = 0.05;
  double wheel_base_ = 0.20;
  double max_linear_ = 0.50;
  double max_angular_ = 1.00;
  double battery_drain_ = 0.02;
  double max_linear_accel_ = 1.00;
  double max_angular_accel_ = 2.00;
  double cg_height_ = 0.15;
  double rollover_safety_factor_ = 0.50;
  double max_lateral_accel_ = 2.00;   // 由 cg_height × 安全系数反推得到
  double max_wheel_speed_ = 20.0;
  double control_rate_ = 50.0;
  double stop_distance_ = 0.15;
  double slow_distance_ = 0.50;
  double emergency_brake_distance_ = 0.30;
  double collision_distance_ = 0.10;
  int turn_direction_ = 1;
  double vehicle_width_ = 0.30;
  double clearance_margin_ = 0.10;
  double gap_pass_speed_ = 0.20;
  double undervoltage_warn_ = 22.0;
  double undervoltage_critical_ = 20.0;
  double overtemp_warn_ = 60.0;
  double overtemp_critical_ = 80.0;
  double cmd_timeout_ = 0.5;

  // 原始目标速度（指令限幅后）与 实际速度（加速度斜坡后）
  double raw_target_vx_ = 0.0;
  double raw_target_wz_ = 0.0;
  double vx_ = 0.0;
  double wz_ = 0.0;
  double left_wheel_ = 0.0;
  double right_wheel_ = 0.0;
  double battery_ = 100.0;

  // 障碍物读数（默认无数据，视为无障碍）
  uint8_t obstacle_type_ = OBSTACLE_LARGE;
  double obstacle_distance_ = std::numeric_limits<double>::infinity();
  double obstacle_height_ = 0.0;
  bool obstacle_valid_ = false;
  bool has_gap_ = false;
  double gap_width_ = 0.0;

  // 驱动器状态读数（默认无数据，视为健康）
  double voltage_ = 0.0;
  double temperature_ = 0.0;
  bool stall_ = false;
  bool driver_fault_ = false;
  bool drive_valid_ = false;

  // 地面摩擦系数（默认 0.6，由 /ground_condition 动态更新）
  double friction_coeff_ = 0.60;
  double traction_margin_ = 0.80;
  double braking_distance_ = 0.15;

  // 报警与故障
  bool alarm_ = false;
  bool hard_stop_ = false;
  uint8_t fault_ = FAULT_NONE;

  // 指令时间戳（看门狗用）
  rclcpp::Time last_cmd_time_;
  bool has_cmd_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<my_custom_msgs::msg::Obstacle>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<my_custom_msgs::msg::DriveStatus>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<my_custom_msgs::msg::GroundCondition>::SharedPtr ground_condition_sub_;
  rclcpp::Publisher<my_custom_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
  return 0;
}
