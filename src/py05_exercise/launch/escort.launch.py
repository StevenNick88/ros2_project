from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关
# from launch.event_handlers import OnProcessStart,OnProcessExit
# from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo
# 获取功能包下share目录路径
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 抽取参数
    escort_back = DeclareLaunchArgument(
        name="turtle_back", default_value="turtle_back")
    # 创建turtlesim_node节点，并生成新乌龟
    master = Node(package="turtlesim", executable="turtlesim_node")
    spawn_back = Node(
        package="py05_exercise",
        executable="exer01_spawn",
        name="spawn_back",
        parameters=[{"x": 2.0, "y": 5.0,
                     "turtle_name": LaunchConfiguration("turtle_back")}]
    )
    # 发布坐标变换
    turtle1_world = Node(
        package="py05_exercise",
        executable="exer02_tf_broadcaster", name="turtle1_world"
    )
    back_world = Node(
        package="py05_exercise",
        executable="exer02_tf_broadcaster",
        name="back_world",
        parameters=[{"turtle": LaunchConfiguration("turtle_back")}]
    )
    escort_goal_back = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="escort_goal_back",
        arguments=["-1.5", "0.0", "0.0", "0.0", "0.0",
                   "0.0", "turtle1", "escort_goal_back"]
    )
    # 监听坐标变换
    back_escort_goal_back = Node(
        package="py05_exercise",
        executable="exer03_tf_listener",
        name="back_escort_goal_back",
        parameters=[{"father_frame": LaunchConfiguration("turtle_back"),
                     "child_frame": "escort_goal_back"}]
    )
    return LaunchDescription([
        escort_back,
        master,
        spawn_back,
        turtle1_world,
        back_world,
        escort_goal_back,
        back_escort_goal_back
    ])
