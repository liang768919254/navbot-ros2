#!/usr/bin/env python3
"""用 slam_toolbox 建图，不启动 Nav2。

建图和导航分成两个 launch 的原因：两者都会发布 map→odom，同时开就是两个节点
抢同一个变换，TF 树会疯狂抖动，表现为 RViz 里机器人位置乱跳。

启动 robot_state_publisher（含 Gazebo 模型）+ slam_toolbox + RViz2。

用法：
  # 终端 1
  ros2 launch navbot_bringup sim_navbot.launch.py
  # 终端 2
  ros2 launch navbot_bringup slam_mapping.launch.py use_sim_time:=true
  # 终端 3：遥控走一圈（0.1 m/s 左右，转太快会建糊）
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # 存图（必须在 slam_toolbox 还活着时存）
  ros2 run nav2_map_server map_saver_cli -f navbot_room

调参重点在 config/slam_toolbox_params.yaml。
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
        description='仿真 true / 真机 false，搞错会导致 TF 全部超时',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_params,
        description='slam_toolbox 参数文件'
    )

    # ---------------------------------------------------------------
    # use_sim_time 要同时注入 slam_toolbox 的参数。
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
