#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using std::placeholders::_1;

//定义节点类
class MinimalSubscriber : public rclcpp::Node
{

public:
    MinimalSubscriber() : Node("student_subcriber")
    {
        subcription = this->create_subscription<Student>("topic_stu", 10,
                                                         std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    rclcpp::Subscription<Student>::SharedPtr subcription;
    // 订阅回调
    void topic_callback(const Student &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "订阅的学生消息：name = %s, age = %d, height = %2.f",
                    msg.name.c_str(), msg.age, msg.height);
    }
};

int main(int argc, char const *argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);
    //调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    //释放资源
    rclcpp::shutdown();

    return 0;
}
