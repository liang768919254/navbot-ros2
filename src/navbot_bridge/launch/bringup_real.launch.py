#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机 bringup：底盘桥 + 雷达 + Nav2（★ 不含 Gazebo，这是它和 sim 版的唯一区别）。

用法：
    ros2 launch navbot_bridge bringup_real.launch.py
    ros2 launch navbot_bridge bringup_real.launch.py use_rviz:=false

★ 真机与仿真的分界（想清楚这一条，面试就答得住）：
    仿真：Gazebo 的 diff_drive 插件**替你**发 odom→base_footprint 的 TF
    真机：没有 Gazebo，这条 TF **只能由 chassis_bridge 发**
    → 所以本文件里 chassis_bridge 是必需的，不是可选的。

★ 雷达驱动：RPLIDAR A1 高速版，走 rplidar_ros 包（Slamtec 官方，需源码编译，见 docs）。
  话题 /scan、frame_id laser_link 与 nav2_params.yaml 对齐。
  底盘串口 /dev/navbot、雷达串口 /dev/rplidar 由 udev 规则固定（见 udev/99-navbot.rules）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("navbot_bridge")
    bringup_share = get_package_share_directory("navbot_bringup")

    bridge_params = os.path.join(bridge_share, "config", "bridge_params.yaml")
    nav_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "navbot_room.yaml")

    # ---- 参数（--ros-args -p）----
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/navbot",
        description="底盘（STM32）串口设备，由 udev 规则固定成 /dev/navbot")
    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port", default_value="/dev/rplidar",
        description="雷达串口设备，由 udev 规则固定成 /dev/rplidar")
    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    # ---- 1. 底盘桥（★ 真机上 odom→base_footprint 的 TF 由它发）----
    bridge_node = Node(
        package="navbot_bridge",
        executable="chassis_bridge",
        name="chassis_bridge",
        output="screen",
        parameters=[bridge_params, {"port": LaunchConfiguration("port")}],
        remappings=[('/cmd_vel', '/cmd_vel_smoothed')],
    )

    # ---- 2. 雷达驱动（RPLIDAR A1 高速版）----
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": 115200,      # A1 固定 115200（SDK 会再握手协商）
            "frame_id": "laser_link",       # ★ 必须和 nav2_params.yaml / URDF 一致
            "angle_compensate": True,       # A1 必须开，否则点云边缘会漂
            # "scan_mode": "Express",       # A1 高速版到货后按需开（见 docs）
        }],
    )
    # ★ 验雷达：ros2 topic hz /scan 应约 10 Hz（协议文档 §六）

    # ---- 3. Nav2 + RViz（复用仿真那套 launch，★ 关键是不传 slam:=True）----
    # ★★ use_sim_time 必须显式给 false：真机没有 /clock，给 true 会让
    #    Nav2 的每一次 tf 查询都超时，症状是「规划器什么都不做」。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "nav_bringup.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": nav_params,
            "use_sim_time": "false",     # ★★ 真机必须 false
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(map_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(bridge_node)
    ld.add_action(lidar_node)
    ld.add_action(nav2)
    return ld
