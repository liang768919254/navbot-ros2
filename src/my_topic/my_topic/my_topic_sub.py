#!/usr/bin/env python3
# -*- coding = UTF-8-*-


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
创建一个订阅者节点
"""

class SubscriberNode(Node):
    def __init__(self,name):
        super().__init__(name)                  #ROS2 节点父类初始化
        self.sub = self.create_subscription(\
            String,"chatter",self.listener_callback,10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.hello()                                        # 接收到话题消息后，调用独立方法hello()
        self.count = 0                                      # 创建一个计数器变量，用于记录接收到的消息数量
    def listener_callback(self,msg):                        # 创建回调函数，执行收到话题消息后对数据的处理
        self.count += 1
        self.get_logger().info('I heard: "%s"(第 %d 次) ' % (msg.data,self.count) )
    def hello(self):
        print("独立方法hello()被调用了！")

def main(args=None):                                        # ROS2节点主入口mmain函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = SubscriberNode("my_topic_sub")                   # 创建节点对象并初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 摧毁节点对象
    rclpy.shutdown()                                        #关闭ROS2 Python接口







