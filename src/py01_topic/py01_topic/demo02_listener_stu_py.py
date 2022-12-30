import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("stu_subscriber_py")
        # 创建订阅方
        self.subscription = self.create_subscription(
            Student,"topic_stu",self.listener_callback,10)
    
    def listener_callback(self,stu):
        self.get_logger().info("py订阅的消息:name=%s, age = %d, height = %.2f" % (stu.name, stu.age,stu.height))

def main(args = None):
    # 初始化ros2客户端
    rclpy.init(args=args)
    # 调用apin函数，并传入节点对象
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()
