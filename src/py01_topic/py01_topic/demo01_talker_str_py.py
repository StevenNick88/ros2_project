import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 定义节点类
class MinimalPublishr(Node):

    def __init__(self):
        super().__init__("minimal_publisher_py")
        # 创建发布方
        self.publisher = self.create_publisher(String,"topic",10)
        # 创建定时器
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        # 计数器
        self.i = 0 
    
    def timer_callback(self):
        msg = String()
        msg.data = "test : %d" % self.i
        self.publisher.publish(msg)
        self.get_logger().info("发布的消息：%s" % msg.data)
        self.i += 1



def main(args = None):
    # 初始化ros2客户端
    rclpy.init(args = args)
    # 调用spin函数，并传入节点对象
    minimal_publisher = MinimalPublishr()
    rclpy.spin(minimal_publisher)
    # 释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()
