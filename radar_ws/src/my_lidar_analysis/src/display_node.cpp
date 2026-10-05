// 节点2：结果接收 + 打印节点
// 只做一件事：订阅 /analysis/result，把节点1 算好的统计结果打印出来
#include <rclcpp/rclcpp.hpp>
#include "my_lidar_analysis/msg/analysis_result.hpp"  // 自定义统计结果消息

using std::placeholders::_1;

class DisplayNode : public rclcpp::Node
{
public:
  DisplayNode()
  : Node("display_node")   // 节点2 的名字
  {
    // 订阅节点1 发布的统计结果
    result_sub_ = this->create_subscription<my_lidar_analysis::msg::AnalysisResult>(
      "/analysis/result", 10,
      std::bind(&DisplayNode::resultCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "节点2(display_node) 已启动，等待统计结果...");
  }

private:
  // 收到统计结果时打印
  void resultCallback(const my_lidar_analysis::msg::AnalysisResult::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "总行程 %.3f m | 最大速度 %.3f m/s | 帧率 点云%d IMU%d Odom%d TF%d Hz",
      msg->total_distance, msg->max_speed, msg->cloud_hz, msg->imu_hz, msg->odom_hz, msg->tf_hz);

    RCLCPP_INFO(this->get_logger(),
      "点云点数: 平均 %.0f | 最小 %u | 最大 %u",
      msg->avg_points, msg->min_points, msg->max_points);

    RCLCPP_INFO(this->get_logger(),
      "IMU 加速度 均值(%.3f, %.3f, %.3f) 噪声σ(%.4f, %.4f, %.4f)",
      msg->imu_mean[0], msg->imu_mean[1], msg->imu_mean[2],
      msg->imu_stddev[0], msg->imu_stddev[1], msg->imu_stddev[2]);

    RCLCPP_INFO(this->get_logger(),
      "IMU 角速度 均值(%.3f, %.3f, %.3f) 噪声σ(%.4f, %.4f, %.4f)",
      msg->imu_mean[3], msg->imu_mean[4], msg->imu_mean[5],
      msg->imu_stddev[3], msg->imu_stddev[4], msg->imu_stddev[5]);
  }

  rclcpp::Subscription<my_lidar_analysis::msg::AnalysisResult>::SharedPtr result_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplayNode>());
  rclcpp::shutdown();
  return 0;
}
