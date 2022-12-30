#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"
#include "turtlesim/srv/spawn.hpp"

using base_interfaces_demo::action::Nav;
using namespace std::chrono_literals;
using namespace std::placeholders;

// 定义节点类
class Exer05ActionClient : public rclcpp::Node
{
public:
    Exer05ActionClient() : Node("exer05_action_client")
    {
        // 创建动作客户端
        nav_client = rclcpp_action::create_client<Nav>(this, "nav");
    }

    // 发送请求数据，并处理服务端响应
    void send_goal(float x, float y, float theta)
    {
        if (!nav_client->wait_for_action_server(5s))
        {
            RCLCPP_INFO(this->get_logger(), "服务连接失败！");
            return;
        }
        // 组织请求数据
        auto goal_msg = Nav::Goal();
        goal_msg.goal_x = x;
        goal_msg.goal_y = y;
        goal_msg.goal_theta = theta;
        rclcpp_action::Client<Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&Exer05ActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&Exer05ActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&Exer05ActionClient::result_callback, this, _1);

        nav_client->async_send_goal(goal_msg, options);
    }

private:
    rclcpp_action::Client<Nav>::SharedPtr nav_client;
    // 处理目标响应
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle)
    {
        // 如果goal_handle是空指针，表示没有请求数据
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标请求被接收");
        }
    }

    // 处理响应的连续反馈
    void feedback_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle,
                           const std::shared_ptr<const Nav::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "距离目标点还有：%.2f 米", feedback->distance);
    }

    // 处理最终响应
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "乌龟最终坐标：(%.2f,%.2f),航向：%.2f",
                        result.result->turtle_x, result.result->turtle_y, result.result->turtle_theta);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "任务被取消");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "任务被中止");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "未知异常");
            break;
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Exer05ActionClient>();
    if (argc != 5)
    {
        RCLCPP_INFO(client->get_logger(), "请传入目标的位姿参数：(x,y,theta)");
        return 1;
    }
    // 发送目标点
    client->send_goal(atof(argv[1]),atof(argv[2]),atof(argv[3]));
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}