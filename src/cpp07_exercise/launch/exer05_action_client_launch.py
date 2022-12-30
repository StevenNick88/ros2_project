from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    x = 8.54
    y = 8.54
    theta = 0.0
    name = "t3"
    # 生成新的乌龟
    spwan = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x':"
             + str(x)+",'y':"+str(y)+",'theta':"+str(theta)+",'name':"+name+"}\""],
        shell=True
    )
    # 创建动作客户端节点
    client = Node(package="cpp07_exercise", executable="exer05_action_client", arguments=[
                  str(x), str(y), str(theta)])
            
    # 怎么控制节点的执行顺序呢？需要通过注册事件来完成
    # 创建事件注册对象，在对象中声明针对哪个目标节点，在哪个事件触发时，执行哪种操作
    register_event = RegisterEventHandler(
        # 创建一个新对象
        event_handler=OnProcessExit( # 触发动作
            target_action=spwan, # 目标节点
            on_exit=client # 触发执行的事件
        )
    )

    return LaunchDescription([spwan, register_event])
