#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash

ros2 launch sensor_bridge sensor.launch.py > /home/liang/ros_study/.check.log 2>&1 &
LPID=$!
sleep 6

echo "===== 1. 话题列表 ====="
ros2 topic list

echo "===== 2. 话题类型（rqt_plot 要知道的字段名看这里）====="
ros2 topic info /my_sensor/v
ros2 interface show std_msgs/msg/Float32

echo "===== 3. 话题上真实的数据 ====="
timeout 4 ros2 topic echo /my_sensor/v 2>&1 | head -6

echo "===== 4. 节点名（ros2 param set 要用这个名字）====="
ros2 node list

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
pkill -f my_sensor 2>/dev/null; pkill -f monitor_node 2>/dev/null
echo DONE
