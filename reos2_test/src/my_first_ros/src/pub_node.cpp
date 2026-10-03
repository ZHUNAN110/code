// pub_node.cpp —— 周期性向 /chatter 话题发布字符串消息
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PubNode : public rclcpp::Node
{
public:
  PubNode() : Node("pub_node")
  {
    // 队列深度 10：来不及处理的消息最多缓存 10 条
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // 每 500 ms 触发一次回调
    timer_ = this->create_wall_timer(
      500ms, [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "pub_node 已启动，发布话题 /chatter");
  }

private:
  void on_timer()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello ROS 2: " + std::to_string(count_++);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "发布: '%s'", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubNode>());
  rclcpp::shutdown();
  return 0;
}
