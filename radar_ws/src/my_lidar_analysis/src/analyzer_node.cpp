// 节点1：数据采集 + 统计节点
// 订阅 5 个话题（点云 / IMU / odom / TF / path），算出统计结果，
// 每 1 秒把结果发布到 /analysis/result 话题（自定义消息 AnalysisResult）
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // 点云消息
#include <sensor_msgs/msg/imu.hpp>           // IMU 消息
#include <nav_msgs/msg/odometry.hpp>         // 里程计消息
#include <nav_msgs/msg/path.hpp>             // 轨迹消息
#include <tf2_msgs/msg/tf_message.hpp>       // TF 消息
#include "my_lidar_analysis/msg/analysis_result.hpp"  // 自定义统计结果消息

#include <cmath>    // sqrt
#include <chrono>   // 定时器

using std::placeholders::_1;

class AnalyzerNode : public rclcpp::Node
{
public:
  AnalyzerNode()
  : Node("analyzer_node")   // 节点1 的名字
  {
    // —— 订阅 5 个原始数据话题 ——
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", 10,
      std::bind(&AnalyzerNode::cloudCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 10,
      std::bind(&AnalyzerNode::imuCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&AnalyzerNode::odomCallback, this, _1));

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      std::bind(&AnalyzerNode::tfCallback, this, _1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&AnalyzerNode::pathCallback, this, _1));

    // —— 发布统计结果（这就是和原来最大的区别）——
    result_pub_ = this->create_publisher<my_lidar_analysis::msg::AnalysisResult>(
      "/analysis/result", 10);

    // 每 1 秒算一次统计并发布
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AnalyzerNode::statsTimer, this));

    RCLCPP_INFO(this->get_logger(), "节点1(analyzer_node) 已启动，正在采集并统计...");
  }

private:
  // 收到点云：数点数，更新最小/最大
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cloud_count_++;
    uint64_t point_count = static_cast<uint64_t>(msg->width) * msg->height;
    total_points_ += point_count;
    if (!has_point_frame_) {
      min_points_ = point_count;
      max_points_ = point_count;
      has_point_frame_ = true;
    } else {
      if (point_count < min_points_) min_points_ = point_count;
      if (point_count > max_points_) max_points_ = point_count;
    }
  }

  // 收到 IMU：累加"和"与"平方和"，之后算均值和噪声σ
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_count_++;
    double v[6] = {
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
    };
    for (int i = 0; i < 6; i++) {
      imu_sum_[i] += v[i];
      imu_sum_sq_[i] += v[i] * v[i];
    }
    imu_samples_++;
  }

  // 收到里程计：累加总行程，并计算速度（找最大速度）
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_count_++;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 当前帧的时间戳（秒 + 纳秒 → 统一转成"秒"）
    double t = static_cast<double>(msg->header.stamp.sec)
             + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    if (have_last_) {
      double dx = x - last_x_;
      double dy = y - last_y_;
      double dz = z - last_z_;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      total_distance_ += dist;

      // 速度 = 位移 / 时间差；记录历史最大速度
      double dt = t - last_t_;
      if (dt > 0.0) {
        double speed = dist / dt;
        if (speed > max_speed_) max_speed_ = speed;
      }
    }
    last_x_ = x;
    last_y_ = y;
    last_z_ = z;
    last_t_ = t;
    have_last_ = true;
  }

  // 收到 TF：只计数（统计帧率用）
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    (void)msg;  // 这里不细看内容，只计数
    tf_count_++;
  }

  // 收到轨迹：这里只确认有数据，不参与统计
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    (void)msg;
  }

  // 每 1 秒：把统计结果填进自定义消息，然后发布出去
  void statsTimer()
  {
    auto result = my_lidar_analysis::msg::AnalysisResult();

    // 总行程（累计，不清零）
    result.total_distance = total_distance_;

    // 最大速度（累计，不清零）
    result.max_speed = max_speed_;

    // 帧率（这一秒的帧数）
    result.cloud_hz = cloud_count_;
    result.imu_hz = imu_count_;
    result.odom_hz = odom_count_;
    result.tf_hz = tf_count_;

    // 点云点数统计
    result.avg_points = (cloud_count_ > 0)
      ? static_cast<double>(total_points_) / cloud_count_ : 0.0;
    result.min_points = static_cast<uint32_t>(min_points_);
    result.max_points = static_cast<uint32_t>(max_points_);

    // IMU 均值和噪声σ
    if (imu_samples_ > 0) {
      for (int i = 0; i < 6; i++) {
        double mean = imu_sum_[i] / imu_samples_;
        double var = imu_sum_sq_[i] / imu_samples_ - mean * mean;
        result.imu_mean[i] = mean;
        result.imu_stddev[i] = (var > 0.0) ? std::sqrt(var) : 0.0;
      }
    }

    // 发布！
    result_pub_->publish(result);

    // 清零，重新数下一秒（注意：total_distance_ 是累计的，不清零）
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

  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // 发布器（自定义消息）
  rclcpp::Publisher<my_lidar_analysis::msg::AnalysisResult>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // 总行程 / 最大速度状态
  double total_distance_ = 0.0;
  double max_speed_ = 0.0;
  bool have_last_ = false;
  double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0;
  double last_t_ = 0.0;   // 上一帧里程计的时间戳（秒）

  // 帧率计数器
  int cloud_count_ = 0;
  int imu_count_ = 0;
  int odom_count_ = 0;
  int tf_count_ = 0;

  // 点云点数统计状态
  uint64_t total_points_ = 0;
  uint64_t min_points_ = 0;
  uint64_t max_points_ = 0;
  bool has_point_frame_ = false;

  // IMU 噪声统计状态
  double imu_sum_[6] = {0.0};
  double imu_sum_sq_[6] = {0.0};
  int imu_samples_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalyzerNode>());
  rclcpp::shutdown();
  return 0;
}
