from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 创建turtlesim_node 节点
    turtle = Node(package="turtlesim",executable="turtlesim_node")
    # 创建测距服务段节点
    sever = Node(package="cpp07_exercise",executable="exer02_sever")

    return LaunchDescription([turtle,sever])