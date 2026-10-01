#!/usr/bin/env python3
"""
nav_bringup.launch.py —— 阶段 D：加载已建好的地图 + AMCL 定位 + Nav2 自主导航

这是 P3 项目的「主 launch」，也是最终演示用的那个。
它做四件事：
  ① 加载地图（map_server 读 .yaml + .pgm）
  ② AMCL 在已知地图上定位（★★ 它负责发布 map→odom 变换）
  ③ 启动 Nav2 全套（规划器 + 控制器 + 代价地图 + 行为树）
  ④ 启动 RViz2（带 Nav2 专用面板，能直接点「Nav2 Goal」发目标点）

★★★ 本文件最重要的一个认知：
    「谁发布 map→odom 变换」只能有一个人。
      建图模式：slam_toolbox 发
      导航模式：AMCL 发
    两个同时开 → TF 抖动 → 机器人位置乱跳 → 规划失败。
    所以本 launch 里绝对不要再 include slam_mapping.launch.py。

------------------------------------------------------------------------------
用法（仿真全流程）：

  # 终端 1：起仿真
  ros2 launch navbot_bringup sim_navbot.launch.py

  # 终端 2：起导航（★ 前提：你已经用 slam_mapping 建好图并 map_saver 存好了）
  ros2 launch navbot_bringup nav_bringup.launch.py use_sim_time:=true

  # 终端 3：在 RViz 里点 "2D Pose Estimate" 给出初始位姿（★ 必须做！）
  #         然后在 RViz 里点 "Nav2 Goal" 给目标点，车自己开过去

  # 或者用命令行直接发目标点（不依赖 RViz）
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0, z: 0.0}, \
      orientation: {w: 1.0}}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory('navbot_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_share, 'maps', 'navbot_room.yaml')
    default_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='地图 yaml 路径（map_saver 生成的那个）'
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 参数文件'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='★★ 仿真 true / 真机 false'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 生命周期节点是否自动激活。调试时设 false 可逐个手动激活'
    )

    # ---------------------------------------------------------------
    # Nav2 官方 bringup
    # ★ 官方 bringup_launch.py 有 slam 参数，我们固定不传（=False），
    #   这样它走「localization_launch.py」分支 —— 也就是 AMCL 定位分支。
    #   这也印证了上一句：slam 和 localization 是二选一的两条路。
    # ---------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            # ★ 改 False：use_composition=True 时 Nav2 节点组合进一个容器，
            #   lifecycle 状态管理会卡死（planner_server 的 ros2 lifecycle get 超时无响应），
            #   导致 autostart 失效、手动激活卡住。False = 独立进程，lifecycle 稳定。
            'use_composition': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'navbot_nav.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        use_rviz_arg,
        autostart_arg,
        nav2,
        rviz_node,
    ])
