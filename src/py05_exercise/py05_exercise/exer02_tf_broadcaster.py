from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf_transformations
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TFDynaBroadcasterPy(Node):
    def __init__(self):
        super().__init__("tf_dyna_broadcaster_py_node_py")
        self.declare_parameter("turtle", "turtle1")
        self.turtle = self.get_parameter(
            "turtle").get_parameter_value().string_value
        # 创建一个动态广播器
        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Pose, "/"+self.turtle+"/pose", self.do_pose, 10)

    # 回调函数中，获取乌龟位姿并生成相对关系然后发布
    def do_pose(self, pose):
        # 组织transformStamped()
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "world"
        ts.child_frame_id = self.turtle
        # 偏移量
        ts.transform.translation.x = pose.x
        ts.transform.translation.y = pose.y
        ts.transform.translation.z = 0.0
        # 四元素
        qtn = tf_transformations.quaternion_from_euler(0.0, 0.0, pose.theta)
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]
        # 发布transform
        self.broadcaster.sendTransform(ts)


def main():
    rclpy.init()
    rclpy.spin(TFDynaBroadcasterPy())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
