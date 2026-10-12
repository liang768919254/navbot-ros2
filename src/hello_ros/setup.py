from setuptools import find_packages, setup

package_name = 'hello_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 这个空文件就是 ament 索引，没有它 ros2 认不出这个包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang768919254@foxmail.com',
    description='Hello ROS 最小示例节点',
    license='MIT',
    entry_points={
        # 左边是可执行名(ros2 run 用)，右边是 包名.模块名:函数名
        'console_scripts': [
            'node_helloros = hello_ros.node_helloros:main',
            'node_helloros_class = hello_ros.node_helloros_class:main', 
            'minimal_publisher   = hello_ros.publisher_member_function:main',
            'minimal_subscription = hello_ros.subscription_member_function:main',
        ],
    },
)
