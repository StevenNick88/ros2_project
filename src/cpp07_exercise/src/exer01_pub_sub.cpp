/*
 需求：订阅乌龟1的位姿信息，解析出线速度和角速度，生成并发布控制乌龟2运动的速度指令

*/

#include "rclcpp/rclcpp.hpp"
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::placeholders;

class Exer01PubSub : public rclcpp::Node
{

public:
  Exer01PubSub() : Node("exer01_pub_sub_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "案例1对象创建!");
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel", 1);
    pose_pub = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 1, std::bind(&Exer01PubSub::poseCallback, this, _1));
  }

private:
  // 根据订阅的乌龟1的速度生成控制乌龟2运动的速度消息并发布
  void poseCallback(const turtlesim::msg::Pose::ConstSharedPtr pose)
  {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = -(pose->angular_velocity); // 角速度取反
    twist.linear.x = pose->linear_velocity;      // 线速度不变
    twist_pub->publish(twist);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_pub;
};

int main(int argc, char const *argv[])
{
  // 初始化ros2客户端
  rclcpp::init(argc, argv);
  // 调用spain函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<Exer01PubSub>());
  // 释放资源
  rclcpp::shutdown();
  return 0;
}
