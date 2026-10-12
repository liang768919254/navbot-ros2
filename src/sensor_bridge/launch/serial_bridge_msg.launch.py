#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge_msg.launch.py —— 阶段三：一次启动「自定义消息版串口桥 + 结构化监控」

放到： ~/ros_study/src/sensor_bridge/launch/serial_bridge_msg.launch.py

用法：
  ros2 launch sensor_bridge serial_bridge_msg.launch.py
  ros2 launch sensor_bridge serial_bridge_msg.launch.py port:=/dev/ttyUSB0 baud:=9600
  ros2 launch sensor_bridge serial_bridge_msg.launch.py frame_id:=imu_link
  ros2 launch sensor_bridge serial_bridge_msg.launch.py port:=/tmp/ttyV1   # 虚拟串口

⚠️ 和阶段二一样：不要把 rqt_plot 塞进来（Qt 程序需要 DISPLAY，
   在 SSH 里会直接报错退出，还会影响 launch 的退出状态）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── 1. 声明 launch 层参数（声明了就必须放进下面的列表）────────
    arg_port = DeclareLaunchArgument(
        'port', default_value='/dev/stm32_bridge',
        description='串口设备路径（用 udev 固定别名，别写 /dev/ttyUSB0）')

    arg_baud = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='波特率，必须和 STM32 侧一致')

    arg_rate = DeclareLaunchArgument(
        'publish_rate', default_value='10.0',
        description='ROS2 侧发布频率 Hz')

    arg_scale = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='value 字段缩放；协议里 value 是整数毫伏时传 1.0')

    arg_frame = DeclareLaunchArgument(
        'frame_id', default_value='sensor_link',
        description='写进 header.frame_id 的坐标名（TF2 查变换用）')

    arg_temp = DeclareLaunchArgument(
        'temp_scale', default_value='0.01',
        description='temperature 字段缩放；协议里温度是 int(℃×100)，故 0.01')

    # ── 2. 桥：注意 executable 是 serial_bridge_msg，不是 serial_bridge ──
    #    type= 这里不写：ROS2 不允许把消息类型当参数传，
    #    类型由 executable 自己决定（这正是"换出口"的做法）。
    bridge = Node(
        package='sensor_bridge',
        executable='serial_bridge_msg',
        name='serial_bridge_msg',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            'scale': ParameterValue(LaunchConfiguration('scale'), value_type=float),
            'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
            'temp_scale': ParameterValue(LaunchConfiguration('temp_scale'), value_type=float),
            'stale_timeout': 1.0,
        }],
    )

    # ── 3. 结构化监控（订阅 SensorData，多通道窗口统计 + 丢帧检测）──
    monitor = Node(
        package='sensor_bridge',
        executable='monitor_msg',
        name='monitor_msg',
        output='screen',
        emulate_tty=True,
        parameters=[{'report_period': 5.0, 'age_warn': 0.5}],
    )

    return LaunchDescription([
        arg_port,
        arg_baud,
        arg_rate,
        arg_scale,
        arg_frame,
        arg_temp,
        bridge,
        monitor,
    ])
