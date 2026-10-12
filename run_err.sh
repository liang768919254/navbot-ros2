#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash
cd /home/liang/ros_study
LOG=/home/liang/ros_study/.err.log

timeout 15 ros2 launch sensor_bridge sensor.launch.py > $LOG 2>&1
echo "===== launch 进程退出码: $? （124 = 被 timeout 杀掉，说明一直在跑）====="
echo
echo "===== 报错行 ====="
grep -n "Traceback\|Error\|error:\|Exception\|raise " $LOG | head -20
echo
echo "===== 日志尾部 ====="
tail -45 $LOG

pkill -f my_sensor 2>/dev/null
pkill -f monitor 2>/dev/null
