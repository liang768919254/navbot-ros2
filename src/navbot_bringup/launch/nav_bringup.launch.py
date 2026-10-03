"""加载已有地图 + AMCL 定位 + Nav2 导航。

前提：地图已经用 slam_mapping 建好并存过。

  # 终端 1
  ros2 launch navbot_bringup sim_navbot.launch.py
  # 终端 2
  ros2 launch navbot_bringup nav_bringup.launch.py use_sim_time:=true
  # 终端 3：RViz 里先点 2D Pose Estimate 给初值，再点 Nav2 Goal

命令行发目标点（不依赖 RViz）：
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0, z: 0.0}, \
      orientation: {w: 1.0}}}}"

map -> odom 只能有一个发布者：建图时是 slam_toolbox，导航时是 AMCL。
两个一起开会 TF 抖动、位置乱跳，所以这个文件不要 include slam_mapping。
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
        description='仿真 true / 真机 false'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 生命周期节点是否自动激活。调试时设 false 可逐个手动激活'
    )

    # 不传 slam 参数（=False），让官方 bringup 走 localization 分支（即 AMCL）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            # 必须 False：use_composition=True 时 Nav2 节点合进一个容器，
            # lifecycle 管理会卡死（planner_server 的 get 无响应），
            # 表现为 autostart 失效、手动激活卡住
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
