import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
import math


class Exer03TFListenerPy(Node):
    def __init__(self):
        super().__init__("exer03_tf_listener_py_node_py")
        # 声明参数
        self.declare_parameter("father_frame", "turtle2")
        self.declare_parameter("child_frame", "turtle1")
        self.father_frame = self.get_parameter(
            "father_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter(
            "child_frame").get_parameter_value().string_value
        # 创建缓存
        self.buffer = Buffer()
        # 创建监听器
        self.listener = TransformListener(self.buffer, self)
        # 创建速度发布方
        self.cmd_pub = self.create_publisher(
            Twist, self.father_frame+"/cmd_vel", 10)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # 实现坐标变换
        if self.buffer.can_transform(self.father_frame, self.child_frame, rclpy.time.Time()):
            ts = self.buffer.lookup_transform(
                self.father_frame, self.child_frame, rclpy.time.Time())
            twist = Twist()
            twist.linear.x = 0.5 * \
                math.sqrt(math.pow(ts.transform.translation.x, 2) +
                          math.pow(ts.transform.translation.y, 2))
            twist.angular.z = 1.0 * \
                math.atan2(ts.transform.translation.y,
                           ts.transform.translation.x)
            self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(Exer03TFListenerPy())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
