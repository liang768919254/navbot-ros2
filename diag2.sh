#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash
cd /home/liang/ros_study

ros2 daemon stop >/dev/null 2>&1
ros2 launch sensor_bridge sensor.launch.py noise:=0.3 > /home/liang/ros_study/.diag2.log 2>&1 &
LPID=$!
sleep 8

echo "===== A. 直接 ros2 node list ====="
timeout 10 ros2 node list 2>&1

echo
echo "===== B. 重启 daemon 之后 ====="
ros2 daemon stop >/dev/null 2>&1
sleep 1
timeout 12 ros2 node list 2>&1

echo
echo "===== C. /my_sensor/v 的发布者数量（应该是 1）====="
ros2 topic info /my_sensor/v 2>&1

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
pkill -f my_sensor 2>/dev/null; pkill -f monitor 2>/dev/null
