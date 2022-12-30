import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 定义节点类
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("stu_publisher_py")
        # 创建发布方
        self.publisher = self.create_publisher(Student,"topic_stu",10)
        # 创建定时器
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        stu = Student()
        stu.name = "李四"
        stu.age = self.i
        stu.height = 1.70
        self.publisher.publish(stu)
        self.get_logger().info("py发布的学生信息:name = %s, age = %d, height = %.2f" % (stu.name,stu.age,stu.height))
        self.i +=1

def main(args = None):
    # 初始化ros2客户端
    rclpy.init(args=args)
    # 调用spin函数，并传入节点对象
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()


