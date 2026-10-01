#!/usr/bin/env python3
"""
sim_navbot.launch.py —— 阶段 B：把 navbot 丢进 Gazebo，能用手柄/键盘开起来。

这一层刻意「不含 Nav2」。
★ 为什么：把「仿真底盘能不能动」和「Nav2 能不能规划」分成两件事验。
  一起起，出错时你分不清是底盘没通还是导航没配。

启动五样：
  ① gzserver + gzclient   Gazebo Classic 11（本机实况，非 Fortress）
  ② robot_state_publisher 模型 → /tf
  ③ spawn_entity         把模型生成到 Gazebo 世界里
  ④ 世界文件             一个空房间 + 几个障碍物
  （/cmd_vel 由你手动发：teleop_twist_keyboard）

------------------------------------------------------------------------------
用法：

  # 1) 起仿真
  ros2 launch navbot_bringup sim_navbot.launch.py

  # 2) 另开一个终端，键盘遥控（第一次开不起来车就看这一步）
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # 3) 验数据流通
  ros2 topic hz /scan          # 应约 10 Hz
  ros2 topic hz /odom          # 应约 50 Hz（插件 update_rate）
  ros2 topic echo /odom --once # 看 pose 里 frame_id 是不是 odom、child 是不是 base_footprint
  ros2 run tf2_tools view_frames   # 应看到 map?--odom--base_footprint--laser_link

★ 注意：本机装的是 Gazebo Classic 11.10.2（`gazebo` 命令），
  不是新版 `gz sim`。两者的 launch 写法完全不同，本文件是 Classic 版。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('navbot_description')
    bringup_share = get_package_share_directory('navbot_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(bringup_share, 'worlds', 'navbot_room.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'navbot.urdf.xacro')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 gzclient（图形界面）。无桌面环境时设 false'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='★ 仿真里必须是 true，否则 TF 时间戳对不上，Nav2 全部超时'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file, description='Gazebo 世界文件路径'
    )

    # ★ 同样不能省 value_type=str（见 display_navbot.launch.py 的注释）
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # ---------------------------------------------------------------
    # ① Gazebo：先起 gzserver + gzclient 的公共入口
    #    用 gazebo_ros 提供的 gzserver.launch.py，比自己写 rosrun 干净
    # ---------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
        }.items(),
    )

    # ---------------------------------------------------------------
    # ② robot_state_publisher
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
    # ③ 把模型生成到 Gazebo 世界里
    #    ★ 两个名字必须成对出现：
    #        -entity      Gazebo 里的模型名
    #        -topic       /robot_description（由上面的 rsp 提供）
    #      写错 topic 的报错是 "Waiting for entity xml on /robot_description"，
    #      然后一直挂着不往下走。
    #    ★ TimerAction 延迟 3 秒：Gazebo 起得慢，
    #      立刻 spawn 会失败并报 "Service /spawn_entity unavailable"。
    # ---------------------------------------------------------------
    spawn_entity_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_navbot',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'navbot',
                    # 出生点：抬高 0.05 避免和地面穿插被弹飞
                    '-x', '0.0', '-y', '0.0', '-z', '0.05',
                ],
            )
        ],
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        world_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
