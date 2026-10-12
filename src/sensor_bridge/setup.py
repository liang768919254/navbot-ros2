import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引，缺了 ros2 就认不出这个包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ── [核心] launch 文件必须"安装"到 share/<包名>/launch/ ──
        #   ros2 launch sensor_bridge sensor.launch.py 找的就是这个位置。
        #   只写文件不加这一行，会报 "file 'sensor.launch.py' was not found"。
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang768919254@foxmail.com',
    description='sensor_bridge —— ROS2 21 讲结业项目：传感器数据进 ROS2 并可视化',
    license='MIT',
    extras_require={
        # 用 extras_require，不要用 tests_require（后者会生成多余的 PATH hook，
        # 导致 ros2 run 报 "No executable found"）
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # ── [核心] 唯一必须手改的地方 ────────────────────────
        #   格式：'可执行名 = 包名.模块名:入口函数'
        #   左边 = ros2 run sensor_bridge <左边>
        #   右边 = 这个可执行文件实际去跑哪个文件的哪个函数
        'console_scripts': [
            'my_sensor = sensor_bridge.fake_sensor_node:main',
            'monitor_node = sensor_bridge.monitor_node:main',
            'stats_node = sensor_bridge.stats_node:main',
            'my_sensor_wave = sensor_bridge.fake_sensor_wave:main',
            # 阶段二：STM32 串口桥（ros2 run sensor_bridge serial_bridge）
            'serial_bridge = sensor_bridge.serial_bridge_node:main',
            # 0D 串口桥最小版（定时器里 readline，用来撞阻塞）
            'my_bridge = sensor_bridge.my_bridge:main',
            # 0D 验收 3：读挪进独立线程的版本
            'my_bridge_threaded = sensor_bridge.my_bridge_threaded:main',
            'serial_bridge_msg = sensor_bridge.serial_bridge_msg_node:main',
            'monitor_msg = sensor_bridge.monitor_msg_node:main',
        ],
    },
)
