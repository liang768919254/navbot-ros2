"""一条命令起全栈：Gazebo + 机器人 + Nav2 +（可选）RViz。

    ros2 launch navbot_bringup sim_all.launch.py                        # 房间图
    ros2 launch navbot_bringup sim_all.launch.py map:=house             # 房子图
    ros2 launch navbot_bringup sim_all.launch.py map:=house gui:=true use_rviz:=true
    ros2 launch navbot_bringup sim_all.launch.py map:=house auto_init:=true

★★ 为什么只暴露 `map` 一个参数，而不是分别传 world / map / params
----------------------------------------------------------------------
换图时这三件套**必须配套**：

    房间 navbot_room  → navbot_room.world  + navbot_room.yaml  + nav2_params.yaml
    房子 navbot_house → navbot_house.world + navbot_house.yaml + nav2_params_house.yaml

只换其中一两个不会报错，但会得到「地图是房子、参数是房间」这种半截状态 ——
表现是路径贴墙、门洞过不去，而你以为是调参问题。一条命令绑死三者，杜绝这种错。

★ 出生点也绑在 map 上：
    房间 (0.0, 0.0)  / 房子 (-1.0, -1.6)
这是 routes_house.json 里 start 的值，也是 S35 跑 6/6 时用的起点。
地图 + 参数 + 出生点三者错一个，6 点巡检的第一段路径就从错误起点算起。

★ 启动顺序：Gazebo 先起，Nav2 延迟 nav_delay 秒再起。
   nav2 一起来就会找 /clock 和 TF，Gazebo 没 ready 时它会疯狂报
   "Timed out waiting for transform"，虽然能自愈但日志一片红，容易误判。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# map 名 → (world, map_yaml, params, x, y)
# ★ 新增地图就在这里加一行，别在下面 if/else 里堆逻辑
MAP_TABLE = {
    'room': {
        'world': 'navbot_room.world',
        'map': 'navbot_room.yaml',
        'params': 'nav2_params.yaml',
        'x': '0.0',
        'y': '0.0',
        'note': '单房间 6×5 m，窄缝 0.655 m',
    },
    'house': {
        'world': 'navbot_house.world',
        'map': 'navbot_house.yaml',
        'params': 'nav2_params_house.yaml',
        'x': '-1.0',
        'y': '-1.6',
        'note': '5 房间 15.2×10.7 m，S35 记录 6/6',
    },
}


def _launch_setup(context, *args, **kwargs):
    """在运行时读 LaunchConfiguration，再决定 include 谁、传什么。"""
    bringup_share = get_package_share_directory('navbot_bringup')

    map_name = LaunchConfiguration('map').perform(context).strip()
    if map_name not in MAP_TABLE:
        raise RuntimeError(
            f"map:={map_name} 不认识，可选：{', '.join(MAP_TABLE)}"
        )
    cfg = MAP_TABLE[map_name]

    world = os.path.join(bringup_share, 'worlds', cfg['world'])
    map_yaml = os.path.join(bringup_share, 'maps', cfg['map'])
    params = os.path.join(bringup_share, 'config', cfg['params'])

    # 允许显式覆盖出生点（默认跟地图走）
    x = LaunchConfiguration('x').perform(context) or cfg['x']
    y = LaunchConfiguration('y').perform(context) or cfg['y']

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_delay = float(LaunchConfiguration('nav_delay').perform(context))
    auto_init = LaunchConfiguration('auto_init')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_navbot.launch.py')
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'yaw': '0.0',
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'autostart': 'true',
        }.items(),
    )

    # AMCL 初值：house 图建议开。不开也能起，但 /map 到 /odom 的变换要等
    # AMCL 自己收敛，期间所有 goal 会被 REJECTED（耗时 0.0 s）。
    set_pose_py = os.path.join(
        os.path.expanduser('~'), 'navbot_ws',
        'tools', 'verify', 'sim_fullchain', 'set_pose.py'
    )
    init_pose = TimerAction(
        period=nav_delay + 25.0,
        actions=[
            LogInfo(msg=f'[sim_all] 注入 AMCL 初值 map=({x}, {y})'),
            ExecuteProcess(
                cmd=['python3', set_pose_py, '--map', x, y, '0.0'],
                output='screen',
            ),
        ],
        condition=IfCondition(auto_init),
    )

    return [
        LogInfo(msg=f'[sim_all] 地图={map_name}（{cfg["note"]}）'),
        LogInfo(msg=f'[sim_all]   world={os.path.basename(world)}'),
        LogInfo(msg=f'[sim_all]   map={os.path.basename(map_yaml)}'),
        LogInfo(msg=f'[sim_all]   params={os.path.basename(params)}'),
        LogInfo(msg=f'[sim_all]   出生点=({x}, {y})，Nav2 延迟 {nav_delay:.0f}s 启动'),
        sim,
        TimerAction(period=nav_delay, actions=[nav]),
        init_pose,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'map', default_value='room',
            description='地图名：room（单房间，默认）| house（5 房间）'
        ),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='是否起 gzclient。无 GPU 的机器必须 false（本机 llvmpipe 会卡死）'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否起 RViz2'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须 true'
        ),
        DeclareLaunchArgument(
            'nav_delay', default_value='12.0',
            description='Gazebo 起来后等多久再起 Nav2（秒）。机器慢就调大'
        ),
        DeclareLaunchArgument(
            'auto_init', default_value='false',
            description='是否自动注入 AMCL 初值（house 图建议 true）'
        ),
        # 空串 = 用地图表里的默认值；填了就覆盖
        DeclareLaunchArgument(
            'x', default_value='',
            description='出生点 x（留空则跟地图默认值）'
        ),
        DeclareLaunchArgument(
            'y', default_value='',
            description='出生点 y（留空则跟地图默认值）'
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
