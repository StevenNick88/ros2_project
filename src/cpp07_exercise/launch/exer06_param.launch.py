from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 创建turtlesim_node节点
    tutle = Node(package="turtlesim", executable="turtlesim_node")
    # 创建背景色修改节点
    param = Node(package="cpp07_exercise", executable="exer06_param_client")

    return LaunchDescription([tutle, param])
