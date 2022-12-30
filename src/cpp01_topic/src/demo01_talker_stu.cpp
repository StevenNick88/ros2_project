#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;

// 定义节点类
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher():Node("student_publisher"),count(0)
    {
        // 创建发布方
        publisher = this->create_publisher<Student>("topic_stu", 10);
        // 创建定时器
        timer = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_back, this));
    }

private:
    void timer_back()
    {
        // 组织消息并发布
        auto stu = Student();
        stu.name = "zhangsan";
        stu.age = count++;
        stu.height = 1.65;
        RCLCPP_INFO(this->get_logger(), "学生信息：name = %s, age = %d, height=%.2f",
                    stu.name.c_str(), stu.age, stu.height);
        publisher->publish(stu);
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Student>::SharedPtr publisher;
    size_t count;
};

int main(int argc, char const *argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc, argv);
    //调用spin()函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    //释放资源
    rclcpp::shutdown();
    return 0;
}
