from setuptools import find_packages, setup

package_name = 'navbot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/bringup_real.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang768919254@foxmail.com',
    description='navbot 底盘桥：STM32 串口协议 <-> ROS2（/odom + TF + /joint_states，/cmd_vel 下行）',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ★ 可执行名必须是 chassis_bridge：
            #   无硬件自测/verify_e2e.sh:382 用它做「写完了没有」的判据，
            #   fake_chassis.py:14 的用法示例也是这个名。
            'chassis_bridge = navbot_bridge.chassis_bridge:main',
        ],
    },
)
