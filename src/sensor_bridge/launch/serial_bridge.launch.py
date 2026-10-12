#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge.launch.py —— 一次启动「串口桥 + 监控节点」

放到： ~/ros_study/src/sensor_bridge/launch/serial_bridge.launch.py
（别忘了 setup.py 的 data_files 里要有 glob('launch/*.launch.py')，
  否则 ros2 launch 会报 file 'serial_bridge.launch.py' was not found）

用法：
  ros2 launch sensor_bridge serial_bridge.launch.py
  ros2 launch sensor_bridge serial_bridge.launch.py port:=/dev/ttyUSB0 baud:=9600
  ros2 launch sensor_bridge serial_bridge.launch.py port:=/tmp/ttyV1          # 虚拟串口

⚠️ 不要把 rqt_plot 塞进这个 launch：
   它是 Qt 程序，需要 DISPLAY。在 SSH 会话里会直接报错退出，
   而且真机上它会影响整个 launch 的退出状态。手动 ros2 run 起更稳。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── 1. 声明 launch 层参数 ─────────────────────────────────
    #    声明了就必须放进 LaunchDescription 的列表，否则
    #    命令行传 xxx:=yyy 会报「未声明的 launch 参数」
    arg_port = DeclareLaunchArgument(
        'port', default_value='/dev/stm32_bridge',
        description='串口设备路径（推荐用 udev 固定别名，别写 /dev/ttyUSB0）')

    arg_baud = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='波特率，必须和 STM32 侧一致')

    arg_rate = DeclareLaunchArgument(
        'publish_rate', default_value='10.0',
        description='ROS2 侧发布频率 Hz')

    arg_scale = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='原始值缩放；MCU 发整数化毫伏时传 0.001')

    # ── 2. 串口桥节点 ────────────────────────────────────────
    # 关于 output（源码依据：launch/actions/execute_process.py 与
    #                  launch/logging/get_output_loggers()）：
    #   'screen'  → stdout + stderr 都打到 launch 所在终端
    #   'log'     → 【默认值】stdout/stderr 写进 launch 主日志文件，
    #                但 stderr 也会上屏
    #   'both'    → 既上屏又写主日志
    #   'own_log' → 各写各自的独立日志文件
    # ⚠️ 关键细节：rclpy 的 get_logger().info() 走的是 stderr，
    #    所以【不写 output='screen' 也能在终端看到日志】。
    #    output='screen' 真正多捞上屏的，是 Python 裸 print() 的 stdout。
    #
    # 关于 emulate_tty：默认 False。False 时子进程 stdout 是管道（块缓冲），
    # 节点里的 print() 会攒够一个缓冲区才吐出来，看起来像"卡住"。
    # 用 get_logger 的话本来就不受影响（Python 3.9+ 的 stderr 恒为行缓冲）。
    bridge = Node(
        package='sensor_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # ParameterValue(..., value_type=xxx) 的作用：
            # 把 launch 传进来的字符串显式转成目标类型，
            # 否则 '115200' 会以字符串形式进参数，ROS2 直接报类型不匹配
            'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            'scale': ParameterValue(LaunchConfiguration('scale'), value_type=float),
            'stale_timeout': 1.0,
        }],
    )

    # ── 3. 监控节点（复用阶段一的 monitor_node）──────────────
    monitor = Node(
        package='sensor_bridge',
        executable='monitor_node',
        name='monitor',
        output='screen',
        parameters=[{'report_period': 2.0}],
    )

    # ── 4. 返回，按顺序执行 ──────────────────────────────────
    return LaunchDescription([
        arg_port,
        arg_baud,
        arg_rate,
        arg_scale,
        bridge,
        monitor,
    ])
