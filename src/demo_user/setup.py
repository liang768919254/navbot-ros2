from setuptools import find_packages, setup

package_name = 'demo_user'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang768919254@foxmail.com',
    description='演示如何在其他包中调用自定义接口',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 格式：'安装后的可执行名 = 包名.文件名:main'
            # 冒号左边是 ros2 run 用的名字，右边必须精确指向那个 main 函数
            'student_publisher = demo_user.student_publisher:main',
            'add_three_ints_server = demo_user.add_three_ints_server:main',
            'demo_client = demo_user.demo_client:main',
        ],
    },
)
