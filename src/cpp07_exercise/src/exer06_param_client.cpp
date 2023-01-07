#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exer06ParamClient : public rclcpp::Node
{
public:
    Exer06ParamClient() : Node("exer06_param_client"), red(0)
    {
        // 创建参数客户端
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }

    // 连接参数服务端
    bool connect_server()
    {
        while (!param_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "终端退出！");
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "参数服务连接中，请稍等。。。");
        }
        return true;
    }

    void update_param()
    {
        red = param_client->get_parameter<int32_t>("background_r");
        // 设置更新参数的频率
        rclcpp::Rate rate(30.0);
        int i = red;
        while (rclcpp::ok())
        {
            i < 255 ? red += 5 : red -= 5;
            i += 5;
            if (i >= 510)
            {
                i = 0;
            }
            param_client->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();
        }
    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client;
    int32_t red;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto param_client = std::make_shared<Exer06ParamClient>();
    if (!param_client->connect_server())
    {
        return 1;
    }
    param_client->update_param();
    // rclcpp::spin(param_client);
    // 5.释放资源
    rclcpp::shutdown();
    return 0;
}
