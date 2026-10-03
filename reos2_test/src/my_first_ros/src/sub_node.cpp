// sub_node.cpp —— 订阅 /cmd_vel 话题并打印收到的速度指令
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class SubNode : public rclcpp::Node
{
public:
  SubNode() : Node("sub_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&SubNode::on_msg, this, _1));

    RCLCPP_INFO(this->get_logger(), "sub_node 已启动，订阅话题 /cmd_vel");
  }

private:
  // 回调只读不改，声明为 const 成员函数
  void on_msg(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(),
      "收到速度指令: 线速度 x=%.2f m/s, 角速度 z=%.2f rad/s",
      msg->linear.x, msg->angular.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubNode>());
  rclcpp::shutdown();
  return 0;
}
