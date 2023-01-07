#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

// 定义节点类
class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
  DynamicFrameBroadcaster() : Node("dynamic_frame_broadcaster")
  {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    std::string topic_name = "/turtle1/pose";

    // 3-2 创建乌龟位姿订阅方
    subscription = this->create_subscription<turtlesim::msg::Pose>(
        topic_name,
        10,
        std::bind(&DynamicFrameBroadcaster::handle_turtle_pose, this, _1));
  }

private:
  // 根据订阅到的乌乌龟位姿生成坐标帧并广播
  void handle_turtle_pose(const turtlesim::msg::Pose &msg)
  {
    // 组织消息
    geometry_msgs::msg::TransformStamped t;
    rclcpp::Time now = this->get_clock()->now(); // 时间缀

    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "turtle1";

    // 平移量
    t.transform.translation.x = msg.x;
    t.transform.translation.y = msg.y;
    t.transform.translation.z = 0.0;

    // 旋转量
    tf2::Quaternion q;
    q.setRPY(0, 0, msg.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 发布消息
    tf_broadcaster->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char const *argv[])
{
  // 初始化ros客户端
  rclcpp::init(argc, argv);
  // 调用spin函数，并传入对象指针
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  // 释放资源
  rclcpp::shutdown();
  return 0;
}
