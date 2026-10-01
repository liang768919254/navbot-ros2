#!/usr/bin/env python3
"""
slam_mapping.launch.py —— 阶段 C：用 slam_toolbox 建图（不启动 Nav2）

★ 为什么建图和导航要分成两个 launch，而不是一个大 launch 带 slam:=true 参数？
  因为「建图」和「导航」是两种完全不同的工作模式：
    建图：slam_toolbox 提供 map→odom 变换，你遥控车走一圈，最后存图
    导航：AMCL 提供 map→odom 变换，加载已存的地图，自己规划路径
  ★★ 两者都要发布 map→odom，同时开 = 两个节点抢同一个变换，
      TF 树会疯狂抖动，表现为 RViz 里机器人位置乱跳。
     这是本项目最高频的「看起来对但不对」的故障。

启动：
  ① robot_state_publisher（含 Gazebo 里的模型）
  ② slam_toolbox（在线异步建图）
  ③ RViz2（配置了 Map + LaserScan + TF）

------------------------------------------------------------------------------
用法（仿真）：

  # 终端 1：起仿真
  ros2 launch navbot_bringup sim_navbot.launch.py

  # 终端 2：起建图
  ros2 launch navbot_bringup slam_mapping.launch.py use_sim_time:=true

  # 终端 3：键盘遥控走一圈（★ 慢一点！0.1 m/s 左右，转太快会建糊）
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # 建完图后，终端 4 存图（★ 必须在 slamtoolbox 还活着时存）
  ros2 run nav2_map_server map_saver_cli -f ~/navbot_map

★ sdf / 参数：建图的调参重点在 config/slam_toolbox_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_share = get_package_share_directory('navbot_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='★★ 仿真必须 true；真机必须 false。搞错的后果是 TF 全部超时',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_params,
        description='slam_toolbox 参数文件'
    )

    # ---------------------------------------------------------------
    # ★ use_sim_time 必须同时注入 slam_toolbox 的参数里。
    #   slam_toolbox 的 online_async_launch.py 会自己声明 use_sim_time，
    #   所以这里不重写参数文件，直接透传给官方 launch。
    # ---------------------------------------------------------------
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'navbot_slam.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        slam_params_arg,
        slam_toolbox,
        rviz_node,
    ])
