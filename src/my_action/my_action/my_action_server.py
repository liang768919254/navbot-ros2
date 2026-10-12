#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 动作示例 —— 负责执行圆周运动动作的服务端

动作（Action）与服务的区别：
  服务  = 一问一答，中间没有进度反馈，耗时任务会把客户端卡住
  动作  = 一目标 + 持续反馈 + 最终结果，且可以中途取消

一个动作由三部分组成（对应 MoveCircle.action 的三段）：
  goal     bool enable   -- 客户端发过来的指令：开始转圈
  result   bool finish   -- 服务端执行完给的回执：是否成功
  feedback int32 state   -- 执行过程中不断汇报的进度：当前转到第几度
"""

import time

import rclpy                                  # ROS2 Python 接口库
from rclpy.node import Node                   # ROS2 节点类
from rclpy.action import ActionServer         # ROS2 动作服务器类
from rclpy.action import CancelResponse       # 取消请求的应答常量
from rclpy.action import GoalResponse         # 目标请求的应答常量

from my_interface.action import MoveCircle    # 自定义的动作接口


class MoveCircleActionServer(Node):
    """圆周运动动作服务端"""

    def __init__(self, name):
        super().__init__(name)

        # 声明参数，方便运行时用 --ros-args -p 调整
        self.declare_parameter('step_degree', 30)      # 每次反馈间隔转过的角度
        self.declare_parameter('step_duration', 0.5)   # 每次反馈间隔的耗时（秒）

        self.step_degree = self.get_parameter('step_degree').value
        self.step_duration = self.get_parameter('step_duration').value

        # 创建动作服务端
        #   第 1 个参数：节点对象，动作挂在这个节点上
        #   第 2 个参数：动作名（客户端按这个名字来找服务端）
        #   第 3 个参数：动作接口类型
        #   第 4 个参数：处理目标请求的回调 —— 决定"接不接这一单"
        #   第 5 个参数：处理取消请求的回调 —— 决定"要不要中止"
        #   第 6 个参数：真正干活的回调 —— 执行流程 + 发反馈 + 返回结果
        self._action_server = ActionServer(
            self,
            MoveCircle,
            'move_circle',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('动作服务端已启动，等待目标……')

    def goal_callback(self, goal_request):
        """
        收到新目标时被调用，决定是否接受这个目标。

        goal_request.enable 是客户端传来的 bool：
          True  -> 接受，开始转圈
          False -> 拒绝，不执行
        """
        self.get_logger().info(f'收到目标请求：enable = {goal_request.enable}')

        if not goal_request.enable:
            self.get_logger().warn('目标中的 enable 为 False，拒绝执行')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        收到取消请求时被调用，决定是否同意取消。

        这里一律同意，实际工程中可能要判断"当前步骤能否安全中断"。
        """
        self.get_logger().info('收到取消请求，同意中止')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        真正执行动作的地方，是服务端的核心。

        执行过程中可以随时通过 goal_handle.publish_feedback() 汇报进度，
        客户端那边就能实时看到圆"转到哪儿了"。
        """
        self.get_logger().info('开始执行圆周运动')

        feedback_msg = MoveCircle.Feedback()

        # 从 0 度转到 360 度，每 step_degree 度汇报一次
        for i in range(0, 360, self.step_degree):
            # 每转一步都检查一下：客户端有没有取消？
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('动作已被取消，提前退出')

                result = MoveCircle.Result()
                result.finish = False
                return result

            feedback_msg.state = i
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'已转到 {i} 度')

            time.sleep(self.step_duration)

        # 转满一圈，正常收尾
        goal_handle.succeed()

        result = MoveCircle.Result()
        result.finish = True
        self.get_logger().info('圆周运动执行完成')

        return result


def main(args=None):
    rclpy.init(args=args)                       # 初始化 ROS2 客户端库

    node = MoveCircleActionServer('move_circle_server')  # 创建服务端节点

    try:
        rclpy.spin(node)                        # 阻塞，持续处理回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()                     # 销毁节点
        rclpy.shutdown()                        # 关闭 ROS2 客户端库


if __name__ == '__main__':
    main()
