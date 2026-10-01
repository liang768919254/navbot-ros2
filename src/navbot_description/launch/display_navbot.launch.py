#!/usr/bin/env python3
"""
display_navbot.launch.py —— 只把模型丢进 RViz 看，不启动仿真、不启动导航。

用途：把 URDF 这一步单独隔离出来验。
★ 这个 launch 属于「阶段 A」，作用是让你在碰 Gazebo / Nav2 之前，
  先用最快的方式确认「坐标系树画对了」。

启动三样：
  ① robot_state_publisher   读模型 → 发 /tf + /tf_static
  ② joint_state_publisher   发全零关节角，让 TF 树不断链
  ③ rviz2                   显示 RobotModel + TF + LaserScan

------------------------------------------------------------------------------
用法：

  ros2 launch navbot_description display_navbot.launch.py

  # 不启动 RViz（只想在终端里看 TF）
  ros2 launch navbot_description display_navbot.launch.py use_rviz:=false

------------------------------------------------------------------------------
RViz 里要看到东西，手动确认这 4 项（配置里已经设好，但你该知道为什么）：
  · Global Options → Fixed Frame = base_footprint
  · RobotModel → Description Topic = /robot_description
  · TF → 能看到 base_footprint→base_link→laser_link 这条链
  · LaserScan → Topic = /scan（本 launch 没有雷达数据，所以这项是空的）
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('navbot_description')

    # ---------------------------------------------------------------
    # 可调参数
    # ---------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='是否使用仿真时间 /clock（本 launch 只看模型，用 false）'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2'
    )

    # ---------------------------------------------------------------
    # ★★ 全项目最容易卡住的一行：
    #    ParameterValue(..., value_type=str) 里的 value_type=str 千万不能省。
    #
    #    不写的话，launch 会把这段 XML 字符串当成 YAML 解析，
    #    于是你得到一堆莫名其妙的 YAML 语法错误，而模型其实毫无问题。
    #    实测报错长这样（Humble 上）：
    #      yaml.parser.ParserError: while parsing a block mapping
    #        expected <block end>, but found '<scalar>'
    #    这是 Humble 上「模型没错却起不来」的头号原因。
    # ---------------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg_share, 'urdf', 'navbot.urdf.xacro'),
        ]),
        value_type=str,
    )

    # ---------------------------------------------------------------
    # ① robot_state_publisher：模型的「翻译官」
    #    读 URDF → 固定关节出 /tf_static，活动关节结合 /joint_states 出 /tf
    # ---------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ---------------------------------------------------------------
    # ② joint_state_publisher（非 GUI 版）
    #    这个模型里只有两个 continuous 轮关节，没有数据源，
    #    所以发全零角度，目的仅仅是让 TF 树不断链。
    #
    #    ★ 接了真机 / 起了 Gazebo 之后，必须把它关掉 ——
    #      否则两个发布者抢同一个 /joint_states 话题，模型会乱抖。
    #      判断依据：ros2 topic info /joint_states -v
    #      的 Publisher count 应该恰好是 1。
    # ---------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        # ★ 必须传 robot_description 参数，否则 joint_state_publisher 会一直
        #   卡在 "Waiting for robot_description to be published on the robot_description topic"
        #   —— 因为 robot_state_publisher 默认只发 TF，不发 /robot_description 话题。
        parameters=[{'robot_description': robot_description}],
    )

    # ---------------------------------------------------------------
    # ③ RViz2
    #    ★ 用 arguments=['-d', 配置路径] 加载 .rviz 配置，不是 parameters！
    #      这是最常写错的地方 —— 写成 parameters 不报错，但配置不生效。
    # ---------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navbot_check.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
