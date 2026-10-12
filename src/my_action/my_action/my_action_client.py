#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 动作示例 —— 发送圆周运动目标的客户端

动作客户端的完整生命周期，一共 5 步：
  1. 创建 ActionClient，等目标服务端上线
  2. send_goal_async()      异步发出目标，返回一个 future
  3. 从结果里拿 goal_handle，判断服务端接没接
  4. 拿到 goal_handle 后，再向它要"结果"的 future
  5. 等结果回来，过程中通过 feedback_callback 实时打印反馈

关键点：动作是异步的，任何一步都是"先拿到一个 future，再等它完成"，
所以代码里会反复出现 add_done_callback / spin_until_future_complete。
"""

import rclpy                                          # ROS2 Python 接口库
from rclpy.node import Node                           # ROS2 节点类
from rclpy.action import ActionClient                 # ROS2 动作客户端类

from my_interface.action import MoveCircle            # 自定义的动作接口


class MoveCircleActionClient(Node):
    """圆周运动动作客户端"""

    def __init__(self, name):
        super().__init__(name)

        # 创建动作客户端：指定节点、动作接口类型、动作名
        # 注意"动作名"必须和服务端创建时用的名字完全一致，否则找不到服务端
        self._action_client = ActionClient(self, MoveCircle, 'move_circle')

        self.get_logger().info('动作客户端已创建')

    def send_goal(self, enable):
        """发送目标，enable 为 True 表示让服务端开始转圈"""
        self.get_logger().info('等待动作服务端上线……')

        # 阻塞等待服务端，超时时间给的 10 秒
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待超时：没有找到名为 move_circle 的动作服务端')
            return

        self.get_logger().info('服务端已上线，开始发送目标')

        goal_msg = MoveCircle.Goal()
        goal_msg.enable = enable

        # 异步发送目标，立刻返回一个 future
        #   第 1 个参数：目标内容
        #   第 2 个参数：收到反馈时的回调（这里注册的是下面的 feedback_callback）
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        # 给 future 挂一个回调：目标被应答时执行 get_goal_result_callback
        self._send_goal_future.add_done_callback(self.get_goal_result_callback)

    def get_goal_result_callback(self, future):
        """收到『服务端是否接受目标』的应答后执行"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('目标被服务端拒绝了')
            return

        self.get_logger().info('目标已被接受，等待最终结果……')

        # 目标被接受后，再问它要"结果"的 future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """收到动作最终结果后执行"""
        result = future.result().result

        self.get_logger().info(f'动作执行结束，finish = {result.finish}')

        rclpy.shutdown()                     # 拿到结果就退出，不等 Ctrl+C

    def feedback_callback(self, feedback_msg):
        """每收到一次反馈就被调用一次，用来实时显示进度"""
        state = feedback_msg.feedback.state
        self.get_logger().info(f'[反馈] 当前已转到 {state} 度')


def main(args=None):
    rclpy.init(args=args)                    # 初始化 ROS2 客户端库

    node = MoveCircleActionClient('move_circle_client')  # 创建客户端节点

    # 发一个 enable=True 的目标：开始转圈
    node.send_goal(True)

    try:
        rclpy.spin(node)                     # 阻塞，持续处理回调
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():                       # 结果回调里可能已经 shutdown 过了
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
