#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; //用于处理时间的

class Talker : public rclcpp::Node
{

public:
  Talker() : Node("talker_node_cpp"), count(0)
  {
    RCLCPP_INFO(this->get_logger(), "create publish node");
    //创建发布方
    publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //创建定时器
    timer = this->create_wall_timer(500ms,std::bind(&Talker::timer_callback,this));

  }

private:
  size_t count;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  //定时器的回调函数
  void timer_callback(){
    auto message = std_msgs::msg::String();
    message.data = "test text"+std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"发布的消息：%s", message.data.c_str());
    publisher->publish(message);
  }
};

int main(int argc, char **argv)
{
  //初始化ros2客户端
  rclcpp::init(argc, argv);
  //调用spin（）函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<Talker>());
  //释放资源
  rclcpp::shutdown();

  return 0;
}
