"""只起仿真，不含 Nav2。

把「底盘能不能动」和「Nav2 能不能规划」分开验，一起起的话出错时
分不清是底盘没通还是导航没配。

  ros2 launch navbot_bringup sim_navbot.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ros2 topic hz /scan    # 约 10 Hz
  ros2 topic hz /odom    # 约 50 Hz

Gazebo 是 Classic 11（gazebo 命令），不是新版 gz sim，
两者 launch 写法完全不同，这个文件是 Classic 版。
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
        description='仿真必须 true，否则 TF 时间戳对不上，Nav2 全部超时'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file, description='Gazebo 世界文件路径'
    )

    # value_type=str 不能省，否则 Command 的输出被当成 YAML 解析
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
    # -entity 和 -topic 必须成对：-entity 是 Gazebo 里的模型名，
    # -topic 指向 robot_state_publisher 提供的 /robot_description。
    # topic 写错的话报错是 "Waiting for entity xml on /robot_description"，
    # 然后一直挂着不往下走。
    # 延迟 3 秒是因为 Gazebo 起得慢，立刻 spawn 会报
    # "Service /spawn_entity unavailable"。
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
