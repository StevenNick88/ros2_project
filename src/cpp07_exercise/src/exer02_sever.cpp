#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace std::placeholders;
// 定义节点类
class Exer02Server : public rclcpp::Node
{

public:
    Exer02Server() : Node("exer02_server"), turtle1_x(0.0), turtle1_y(0.0)
    {
        pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&Exer02Server::poseCallBack, this, _1));
        distance_server = this->create_service<base_interfaces_demo::srv::Distance>(
            "distance", std::bind(&Exer02Server::distanceCallback, this, _1, _2));
    }

private:
    void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose)
    {
        turtle1_x = pose->x;
        turtle1_y = pose->y;
    }
    // 解析目标值，计算距离并反馈结果
    void distanceCallback(const base_interfaces_demo::srv::Distance_Request::SharedPtr request,
                          base_interfaces_demo::srv::Distance_Response::SharedPtr response)
    {
        // 1 解析目录坐标
        float goal_x = request->x;
        float goal_y = request->y;
        // 2 距离计算
        float x = goal_x - turtle1_x;
        float y = goal_y - turtle1_y;
        // 3 将结果设置到响应
        response->distance = std::sqrt(x * x + y * y);
        RCLCPP_INFO(this->get_logger(), "目标坐标：(%.2f,%.2f),距离：%.2f", goal_x, goal_y, response->distance);
    }

    float turtle1_x;
    float turtle1_y;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Service<base_interfaces_demo::srv::Distance>::SharedPtr distance_server;
};

int main(int argc, char const *argv[])
{
    // 初始化ros2客户端
    rclcpp::init(argc, argv);
    // 调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<Exer02Server>());
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
