from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引，缺了 ros2 认不出这个包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang768919254@foxmail.com',
    description='imu_driver —— 基于 ROS2 核心概念模板库生成',
    license='MIT',
    extras_require={
        # ★ 必须用 extras_require，不能用 tests_require
        #   tests_require 会触发额外构建路径，生成多余的 PATH hook，
        #   导致 ros2 run 报 "No executable found"
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher = imu_driver.publisher:main',
            'subscriber = imu_driver.subscriber:main',
            'processor = imu_driver.processor:main',
        ],
    },
)
