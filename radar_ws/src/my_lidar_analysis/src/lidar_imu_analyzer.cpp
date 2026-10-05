// 订阅 Mid-360 的点云和 IMU，以及 fast_lio 输出的 odom / TF，打印收到的数据信息
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // 点云消息
#include <sensor_msgs/msg/imu.hpp>           // IMU 消息
#include <nav_msgs/msg/odometry.hpp>         // 里程计消息
#include <nav_msgs/msg/path.hpp>             // 轨迹消息
#include <tf2_msgs/msg/tf_message.hpp>       // TF 消息


#include <cmath>                             // sqrt 求平方根
#include <chrono>                            // 定时器用到的时间

// 这句话是固定的：让 std::placeholders::_1 可以直接写 _1
using std::placeholders::_1;

// 节点类：继承 rclcpp::Node
class LidarImuAnalyzer : public rclcpp::Node
{
public:
  LidarImuAnalyzer()
  : Node("lidar_imu_analyzer")   // 节点名字
  {
    // 订阅点云频道（话题名，队列长度，回调函数）
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", 10,
      std::bind(&LidarImuAnalyzer::cloudCallback, this, _1));

    // 订阅 IMU 频道
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 10,
      std::bind(&LidarImuAnalyzer::imuCallback, this, _1));

    // 订阅里程计（fast_lio 发布的位置/姿态）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&LidarImuAnalyzer::odomCallback, this, _1));

    // 订阅 TF（fast_lio 广播的坐标系变换）
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      std::bind(&LidarImuAnalyzer::tfCallback, this, _1));

    // 订阅轨迹（fast_lio 输出的历史位姿序列）
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&LidarImuAnalyzer::pathCallback, this, _1));

    // 定时器：每 1 秒自动调用 statsTimer，打印帧率统计
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LidarImuAnalyzer::statsTimer, this));

    RCLCPP_INFO(this->get_logger(), "节点已启动，正在等待数据...");
  }




private:
  // 收到点云时会自动调用这个函数
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cloud_count_++;  // 每收到一帧点云，计数器 +1

    // 点数 = 宽度 × 高度（一帧点云通常 width=点数, height=1）
    uint64_t point_count = static_cast<uint64_t>(msg->width) * msg->height;

    // 累加总点数，并更新最小/最大点数
    total_points_ += point_count;
    if (!has_point_frame_) {
      min_points_ = point_count;
      max_points_ = point_count;
      has_point_frame_ = true;
    } else {
      if (point_count < min_points_) min_points_ = point_count;
      if (point_count > max_points_) max_points_ = point_count;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "收到点云: %lu 个点 | 帧时间戳 %u.%u | 数据 %zu 字节",
      point_count, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->data.size());
  }





  // 收到 IMU 时会自动调用这个函数
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_count_++;  // 每收到一帧 IMU，计数器 +1

    // 把 6 个轴的值放进数组：0-2 是加速度 xyz，3-5 是角速度 xyz
    double v[6] = {
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
    };
    // 累加"和"与"平方和"，之后就能算出均值和标准差
    for (int i = 0; i < 6; i++) {
      imu_sum_[i] += v[i];
      imu_sum_sq_[i] += v[i] * v[i];
    }
    imu_samples_++;

    RCLCPP_INFO(
      this->get_logger(),
      "收到 IMU: 线加速度 x=%.3f y=%.3f z=%.3f | 角速度 x=%.3f y=%.3f z=%.3f",
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  }



//计算总行程
  // 收到里程计时会自动调用这个函数
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_count_++;  // 每收到一帧里程计，计数器 +1

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 如果不是第一帧，就累加“这一帧到上一帧”的直线距离
    if (have_last_) {
      double dx = x - last_x_;
      double dy = y - last_y_;
      double dz = z - last_z_;
      total_distance_ += std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 记住这一帧的位置，供下一帧计算
    last_x_ = x;
    last_y_ = y;
    last_z_ = z;
    have_last_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Odom 位置 x=%.3f y=%.3f z=%.3f | 总行程 %.3f m",
      x, y, z, total_distance_);
  }




  // 收到 TF 时会自动调用这个函数
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf_count_++;  // 每收到一条 TF 消息，计数器 +1

    // 一条 TF 消息里可能包含多个变换，逐个打印
    for (const auto & t : msg->transforms)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "收到 TF: %s -> %s | 平移 x=%.3f y=%.3f z=%.3f",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    }
  }

  // 收到轨迹时会自动调用这个函数
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到 Path: 共 %zu 个位姿点", msg->poses.size());

    // 打印最新一个点的位置（轨迹最后一个点）
    if (!msg->poses.empty()) {
      const auto & last = msg->poses.back().pose.position;
      RCLCPP_INFO(
        this->get_logger(),
        "  最新点位置 x=%.3f y=%.3f z=%.3f", last.x, last.y, last.z);
    }
  }



  // 每 1 秒被定时器调用一次：打印这一秒里各频道收到多少帧，然后清零
  void statsTimer()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "帧率统计: 点云 %d Hz | IMU %d Hz | Odom %d Hz | TF %d Hz",
      cloud_count_, imu_count_, odom_count_, tf_count_);

    // 点云点数统计（这一秒收到过点云才打印）
    if (cloud_count_ > 0) {
      double avg = static_cast<double>(total_points_) / cloud_count_;
      RCLCPP_INFO(
        this->get_logger(),
        "点云点数: %d 帧 | 平均 %.0f 点/帧 | 最小 %lu | 最大 %lu",
        cloud_count_, avg, min_points_, max_points_);
    }




    // IMU 噪声统计（这一秒收到过 IMU 才打印）
    if (imu_samples_ > 0) {
      double mean[6], stddev[6];
      for (int i = 0; i < 6; i++) {
        mean[i] = imu_sum_[i] / imu_samples_;                          // 均值
        double var = imu_sum_sq_[i] / imu_samples_ - mean[i] * mean[i];  // 方差
        stddev[i] = (var > 0.0) ? std::sqrt(var) : 0.0;                // 标准差
      }
      RCLCPP_INFO(
        this->get_logger(),
        "IMU 加速度  均值(%.3f, %.3f, %.3f)  噪声σ(%.4f, %.4f, %.4f)",
        mean[0], mean[1], mean[2], stddev[0], stddev[1], stddev[2]);
      RCLCPP_INFO(
        this->get_logger(),
        "IMU 角速度  均值(%.3f, %.3f, %.3f)  噪声σ(%.4f, %.4f, %.4f)",
        mean[3], mean[4], mean[5], stddev[3], stddev[4], stddev[5]);
    }



    
    // 清零，重新数下一秒
    cloud_count_ = 0;
    imu_count_ = 0;
    odom_count_ = 0;
    tf_count_ = 0;
    total_points_ = 0;
    has_point_frame_ = false;
    imu_samples_ = 0;
    for (int i = 0; i < 6; i++) {
      imu_sum_[i] = 0.0;
      imu_sum_sq_[i] = 0.0;
    }
  }




  // 订阅器（成员变量，防止被提前销毁）
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // 累计总行程用的状态（在多次回调之间“记住”上一次的位置）
  double total_distance_ = 0.0;
  bool have_last_ = false;
  double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0;

  // 帧率统计用的计数器 + 定时器
  int cloud_count_ = 0;
  int imu_count_ = 0;
  int odom_count_ = 0;
  int tf_count_ = 0;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // 点云点数统计用的状态
  uint64_t total_points_ = 0;
  uint64_t min_points_ = 0;
  uint64_t max_points_ = 0;
  bool has_point_frame_ = false;

  // IMU 噪声统计用的状态（数组 0-2 是加速度，3-5 是角速度）
  double imu_sum_[6] = {0.0};     // 累加和
  double imu_sum_sq_[6] = {0.0};  // 累加平方和
  int imu_samples_ = 0;           // 样本数
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                  // 初始化 ROS
  rclcpp::spin(std::make_shared<LidarImuAnalyzer>());        // 让节点一直运行，处理回调
  rclcpp::shutdown();                                        // 结束清理
  return 0;
}
