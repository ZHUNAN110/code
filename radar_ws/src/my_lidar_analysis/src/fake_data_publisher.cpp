// 节点3：假数据发布器（测试用）
// 不发真的雷达数据，而是自己造假的点云 / IMU / odom / TF / path，
// 让节点1 和节点2 在没有真实雷达的情况下也能跑起来看效果。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <cstdlib>
#include <vector>
#include <chrono>

class FakeDataPublisher : public rclcpp::Node
{
public:
  FakeDataPublisher()
  : Node("fake_data_publisher")
  {
    // —— 发布 5 个话题，正好对应节点1 订阅的 5 个 ——
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/livox/imu", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/Odometry", 10);
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/path", 10);

    // 慢一点的定时器：10Hz，发点云 + odom + tf + path（模拟雷达/里程计的频率）
    slow_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FakeDataPublisher::slowTick, this));

    // 快一点的定时器：100Hz，发 IMU（模拟 IMU 的高频率）
    fast_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&FakeDataPublisher::imuTick, this));

    std::srand(42);  // 固定随机种子，让每次运行的数据一样（方便对比）

    RCLCPP_INFO(this->get_logger(), "节点3(fake_data_publisher) 已启动，开始造假数据...");
  }

private:
  // 生成一小段随机噪声（范围约 ±0.01）
  double noise()
  {
    return (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 0.02;
  }

  // 造一帧点云：N 个随机点
  sensor_msgs::msg::PointCloud2 makeCloud(int n)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "livox_frame";
    cloud.header.stamp = this->now();
    cloud.height = 1;
    cloud.width = n;

    // 定义每个点的字段：x, y, z（都是 float32）
    sensor_msgs::msg::PointField fx, fy, fz;
    fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    cloud.fields = {fx, fy, fz};

    cloud.point_step = 12;              // 每个点 3 个 float32 = 12 字节
    cloud.row_step = cloud.point_step * n;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);

    // 填随机点（前方一个范围里）
    float * p = reinterpret_cast<float *>(cloud.data.data());
    for (int i = 0; i < n; i++) {
      p[i * 3 + 0] = 2.0f + 8.0f * (static_cast<float>(std::rand()) / RAND_MAX);  // x: 2~10
      p[i * 3 + 1] = -5.0f + 10.0f * (static_cast<float>(std::rand()) / RAND_MAX); // y: -5~5
      p[i * 3 + 2] = -2.0f + 4.0f * (static_cast<float>(std::rand()) / RAND_MAX);  // z: -2~2
    }
    return cloud;
  }

  // 10Hz：发点云、odom、tf、path
  void slowTick()
  {
    // 1) 点云（每次点数在 19000~21000 之间波动，模拟真实变化）
    int n = 20000 + (std::rand() % 2000) - 1000;
    cloud_pub_->publish(makeCloud(n));

    // 2) odom：让机器人沿半径 2m 的圆走，这样总行程会持续增长
    angle_ += 0.05;   // 每 100ms 转 0.05 rad
    double x = 2.0 * std::cos(angle_);
    double y = 2.0 * std::sin(angle_);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.header.stamp = this->now();
    odom.child_frame_id = "body";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;  // 姿态先不管，设单位四元数
    odom_pub_->publish(odom);

    // 3) tf：camera_init -> body，跟着 odom 走
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped ts;
    ts.header.frame_id = "camera_init";
    ts.header.stamp = this->now();
    ts.child_frame_id = "body";
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.w = 1.0;
    tf_msg.transforms.push_back(ts);
    tf_pub_->publish(tf_msg);

    // 4) path：把走过的位置一个个存起来，越走越长
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    path_msg_.header = odom.header;
    path_msg_.poses.push_back(ps);
    path_pub_->publish(path_msg_);
  }

  // 100Hz：发 IMU（带重力 + 噪声）
  void imuTick()
  {
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "body";
    imu.header.stamp = this->now();

    // 静止时加速度只有重力（z 方向约 9.81），再叠加上噪声
    imu.linear_acceleration.x = 0.0 + noise();
    imu.linear_acceleration.y = 0.0 + noise();
    imu.linear_acceleration.z = 9.81 + noise();

    // 角速度：机器人转圈时 z 轴有恒定角速度，加一点噪声
    imu.angular_velocity.x = noise();
    imu.angular_velocity.y = noise();
    imu.angular_velocity.z = 0.5 + noise();

    imu_pub_->publish(imu);
  }

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr slow_timer_;
  rclcpp::TimerBase::SharedPtr fast_timer_;

  // 机器人转圈的当前角度、走过的轨迹
  double angle_ = 0.0;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDataPublisher>());
  rclcpp::shutdown();
  return 0;
}
