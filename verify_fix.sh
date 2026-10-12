#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash
cd /home/liang/ros_study
LOG=/home/liang/ros_study/.after.log

ros2 daemon stop >/dev/null 2>&1
ros2 launch sensor_bridge sensor.launch.py noise:=0.3 > $LOG 2>&1 &
LPID=$!
sleep 8

echo "===== 1. 节点列表（修好后应该只有 2 个，且无同名警告）====="
timeout 10 ros2 node list 2>&1

echo
echo "===== 2. 两个话题的发布者数量（都应该 = 1）====="
ros2 topic info /my_sensor/v
echo "---"
ros2 topic info /my_sensor/noise

echo
echo "===== 3. 日志里还有几条 WARN ====="
grep -c "WARN" $LOG

echo
echo "===== 4. monitor 的汇报（噪音应该不再是 0）====="
grep "噪音" $LOG | tail -3

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
pkill -f my_sensor 2>/dev/null; pkill -f monitor 2>/dev/null
