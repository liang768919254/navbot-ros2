#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
演示：在其他包里调用自定义接口（消息 msg）

这个包叫 demo_user，接口定义在另一个包 demo_interface 里。
跨包调用的关键就一行 import：
    from demo_interface.msg import Student
        └─ 接口包名 ─┘ └类型┘ └─ 消息名 ─┘

前提是 demo_interface 已经构建并 source 过，否则这行 import 会报
ModuleNotFoundError: No module named 'demo_interface'。
"""

import rclpy                                 # ROS2 Python 接口库
from rclpy.node import Node                  # ROS2 节点类

from demo_interface.msg import Student       # ← 跨包调用自定义消息


class StudentPublisher(Node):
    """把学生信息以话题的形式周期性发出去"""

    def __init__(self, name):
        super().__init__(name)

        # 创建发布者：话题名 student_info，消息类型 Student，队列长度 10
        self.publisher_ = self.create_publisher(Student, 'student_info', 10)

        # 定时器：每 1 秒触发一次 send_message
        self.timer = self.create_timer(1.0, self.send_message)

        # 用一个计数器模拟"不同的学生"，让每次发的内容不一样
        self.count = 0

        self.get_logger().info('学生信息发布者已启动，开始发送数据……')

    def send_message(self):
        """组装一条 Student 消息并发出去"""
        msg = Student()                      # 实例化自定义消息

        # 点号访问字段 —— 字段名必须和 Student.msg 里定义的完全一致
        msg.name = f'小梁同学{self.count}'
        msg.age = 20 + self.count % 5
        msg.score = 88.5 + self.count
        msg.courses = ['信号与系统', '嵌入式Linux', 'ROS2']   # 变长数组直接赋 Python list

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'已发送 → 姓名={msg.name}, 年龄={msg.age}, '
            f'成绩={msg.score}, 课程={msg.courses}'
        )

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = StudentPublisher('student_publisher')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
