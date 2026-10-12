#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash
cd /home/liang/ros_study
LOG=/home/liang/ros_study/.diag.log

ros2 launch sensor_bridge sensor.launch.py noise:=0.3 > $LOG 2>&1 &
LPID=$!
sleep 7

echo "===== 1. 图里到底有几个节点 ====="
ros2 node list

echo
echo "===== 2. 话题列表 ====="
ros2 topic list

echo
echo "===== 3. /my_sensor/noise 有几个发布者 ====="
ros2 topic info /my_sensor/noise

echo
echo "===== 4. /my_sensor/noise 上的真实数据 ====="
timeout 4 ros2 topic echo /my_sensor/noise 2>&1 | head -6

echo
echo "===== 5. 日志里的 WARN / 报错 ====="
grep -n "WARN\|Error\|Traceback\|Exception" $LOG | head -10

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
pkill -f my_sensor 2>/dev/null; pkill -f monitor 2>/dev/null
