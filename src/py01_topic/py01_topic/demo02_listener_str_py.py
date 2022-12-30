import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 定义节点类
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subcriber_py")
        # 创建订阅方
        self.subscription = self.create_subscription(String,"topic",self.listener_callback,10)

    def listener_callback(self,msg):
        self.get_logger().info("订阅的消息：%s" % msg.data)


def main(args = None):
    # 初始化ros2客户端
    rclpy.init(args=args)
    # 调用spin函数，并传入节点对象
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # 释放资源
    rclpy.shutdown()

if __name__ =="__main__":
    main()

