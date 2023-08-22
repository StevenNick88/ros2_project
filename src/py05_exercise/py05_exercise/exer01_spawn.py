import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class Exer01SpawnPy(Node):
   def __init__(self):
       super().__init__("exer01_spawn_py_node_py")
       # 声明参数
       self.declare_parameter("x",3.5)
       self.declare_parameter("y",3.5)
       self.declare_parameter("theta",1.57)
       self.declare_parameter("turtle_name","turtle2")
       self.x = self.get_parameter("x").get_parameter_value().double_value
       self.y = self.get_parameter("y").get_parameter_value().double_value
       self.theta = self.get_parameter("theta").get_parameter_value().double_value
       self.turtle_name = self.get_parameter("turtle_name").get_parameter_value().string_value
       # 创建服务客户端
       self.client = self.create_client(Spawn,"/spawn")
       # 连接服务端
       while not self.client.wait_for_service(1.0):
          self.get_logger().info("服务连接中")
       
   # 组织并发送数据    
   def request(self):
       # 组织数据
       request = Spawn.Request()
       request.x = self.x
       request.y = self.y
       request.theta = self.theta
       request.name = self.turtle_name
       # 发送数据
       self.future = self.client.call_async(request)
def main(): 
   rclpy.init()
   # 创建自定义节点类对象，组织函数，处理相应结果
   spawn = Exer01SpawnPy()
   spawn.request()
   # 这里会一直等待执行直到spawn.future里面有数据
   rclpy.spin_until_future_complete(spawn,spawn.future)
   # 处理响应结果
   response = spawn.future.result()
   # 表示没有生成新乌龟，所以len = 0
   if len(response.name) ==0:
      spawn.get_logger().info("乌龟重名了")
   else:
      spawn.get_logger().info("乌龟生成了")
   rclpy.shutdown()

if __name__=="__main__":
   main() 
