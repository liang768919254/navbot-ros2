#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liang/ros_study/install/setup.bash
cd /home/liang/ros_study

echo "########## 0. random.gauss(0.0, 0.0) 到底返回什么 ##########"
/usr/bin/python3 -c "import random; print([random.gauss(0.0, 0.0) for _ in range(6)])"
echo "           对比 random.gauss(0.0, 0.3)："
/usr/bin/python3 -c "import random; print([round(random.gauss(0.0, 0.3), 3) for _ in range(6)])"
echo

echo "########## A. 不传 noise（用默认值 0.0）##########"
ros2 launch sensor_bridge sensor.launch.py > .noiseA.log 2>&1 &
sleep 7
echo "-- 节点里 noise 参数当前值 --"
ros2 param get /fake_sensor noise
echo "-- monitor 的汇报 --"
grep "噪音" .noiseA.log | tail -3
pkill -f "ros2 launch" >/dev/null 2>&1; pkill -f my_sensor >/dev/null 2>&1; pkill -f monitor_node >/dev/null 2>&1
sleep 3

echo
echo "########## B. 传 noise:=0.3 ##########"
ros2 launch sensor_bridge sensor.launch.py noise:=0.3 > .noiseB.log 2>&1 &
sleep 7
echo "-- 节点里 noise 参数当前值 --"
ros2 param get /fake_sensor noise
echo "-- monitor 的汇报 --"
grep "噪音" .noiseB.log | tail -3
pkill -f "ros2 launch" >/dev/null 2>&1; pkill -f my_sensor >/dev/null 2>&1; pkill -f monitor_node >/dev/null 2>&1
sleep 1
echo "DONE"
