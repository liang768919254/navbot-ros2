from setuptools import find_packages, setup

package_name = 'my_topic'

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
    description='话题练习包',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 左边是可执行名(ros2 run 用)，右边是 包名.模块名:函数名
            'my_topoc = my_topic.my_topoc:main',
            'my_topic_pub = my_topic.my_topic_pub:main',
            'my_topic_sub = my_topic.my_topic_sub:main',
        ],
    },
)
