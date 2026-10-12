#!/usr/bin/env python3
# -*- coding = UTF-8-*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
ROS2 官方节点示例--发布者
"""

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String,'chatter',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS "%d"' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


