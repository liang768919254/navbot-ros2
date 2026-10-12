#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sensor.launch.py —— 一次启动 sensor_bridge 阶段 1 的两个节点

【launch 文件解决什么问题】
真实机器人有几十个节点，不可能开几十个终端手动 ros2 run。
launch 就是"编排"：一次命令、一套配置、全部起来。

【参数的三层传递】——这是阶段 1 真正要理解的东西
    命令行  ros2 launch ... amplitude:=5.0
       ↓   DeclareLaunchArgument 声明 + LaunchConfiguration 取值
    launch 层
       ↓   写进 Node(parameters=[...])
    节点层  declare_parameter / get_parameter
也就是说：改一个数，不碰代码、不重新编译。

用法：
    ros2 launch sensor_bridge sensor.launch.py
    ros2 launch sensor_bridge sensor.launch.py amplitude:=5.0 freq:=0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── ① 声明 launch 层参数 ──────────────────────────────
    # 默认值必须和节点里 declare_parameter 的默认值是同一数量级，
    # 否则"不传参数"时的行为和"单独 ros2 run"不一致，很容易查错。
    arg_amplitude = DeclareLaunchArgument(
        'amplitude', default_value='1.0', description='正弦幅值')
    arg_freq = DeclareLaunchArgument(
        'freq', default_value='0.5', description='正弦频率 Hz')
    arg_rate = DeclareLaunchArgument(
        'rate', default_value='10.0', description='发布频率 Hz')
    arg_noise = DeclareLaunchArgument(
        'noise', default_value='0.0', description='模拟噪音 DB')
    arg_wave = DeclareLaunchArgument(
        'wave', default_value='sin', description='波形 sin/square/saw')
    # ── ② 假传感器节点 ────────────────────────────────────
    my_sensor = Node(
        package='sensor_bridge',
        executable='my_sensor',     # 对应 setup.py 里的 entry_points 名字
        name='fake_sensor',                # 运行时的节点名（可和 executable 不同）
        output='screen',                   # 日志打到当前终端
        parameters=[{
            # [坑] 关键中的关键：launch 命令行传进来的东西本质是**字符串**，
            #      而节点里 declare_parameter('amplitude', 1.0) 声明的是 double。
            #      直接写 LaunchConfiguration('amplitude') 会报
            #      "parameter has invalid type: expected [double] got [string]"。
            #      用 ParameterValue(..., value_type=float) 做类型转换。
            'amplitude': ParameterValue(LaunchConfiguration('amplitude'),
                                        value_type=float),
            'freq': ParameterValue(LaunchConfiguration('freq'),
                                   value_type=float),
            'rate': ParameterValue(LaunchConfiguration('rate'),
                                   value_type=float),
            'noise': ParameterValue(LaunchConfiguration('noise'),
                                    value_type=float),
            'wave': ParameterValue(LaunchConfiguration('wave'),
                                   value_type=str),
        }],
    )

    # ── ③ 监视器节点 ──────────────────────────────────────
    # 它不需要命令行配置，直接写死一个数字即可（这时不用 ParameterValue，
    # 因为 Python 的 2.0 本来就是 float，不存在类型不匹配问题）。
    monitor = Node(
        package='sensor_bridge',
        executable='monitor_node',
        name='monitor',
        output='screen',
        parameters=[{'report_period': 2.0}],
    )

    # ── ④ 返回：所有 launch 动作会被依次执行 ──────────────
    return LaunchDescription([
        arg_amplitude,
        arg_freq,
        arg_rate,
        arg_noise,
        arg_wave,
        my_sensor,
        monitor,
    ])
