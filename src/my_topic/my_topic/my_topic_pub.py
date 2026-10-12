#!/usr/bin/env python3
# -*- coding = UTF-8-*-


"""
ROS2--话题练习
发布“Hello Wrold”话题
"""

import rclpy                        #ROS2 Python接口库
from rclpy.node import Node         #ROS2 节点类
from std_msgs.msg   import String   # 字符串消息类型

"""
创建一个发布者节点
"""

class PublisherNode(Node):
    def __init__(self,name):
        super().__init__(name)

        self.pub = self.create_publisher(String,"chatter",10)       #创建发布者对象（消息类型，话题名、队列长度）
        self.timer = self.create_timer(2,self.timer_callback)       #创建一个定时器（单位为秒的周期，定时执行的回调函数）    
        self.hello()
        self.count = 0
    def timer_callback(self):
        print("计数器count的值为:",self.count)
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info('Publishing: "%s"(第 %d 次)'% (msg.data,self.count))
    def hello(self):
        print("独立方法hello()被调用了！")

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("my_topic_pub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()





