from setuptools import find_packages, setup

package_name = 'my_action'

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
    description='练习包',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_action = my_action.my_action:main',
            'my_action_server = my_action.my_action_server:main',
            'my_action_client = my_action.my_action_client:main',
        ],
    },
)
