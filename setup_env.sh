#!/usr/bin/env bash
# ============================================================================
# setup_env.sh —— navbot ROS2 Humble 环境一键重建（魔搭免费实例专用）
# ============================================================================
# 场景：魔搭免费实例关机/重置后，环境层（apt 包、系统配置）被清空，
#       但 /mnt/workspace 是持久化 NAS（源码不丢）。
#       本脚本只负责重建"环境层"，跑一遍约 15~25 分钟。
#
# 用法：
#   cd /mnt/workspace
#   bash setup_env.sh
#
# 特性：幂等（可重复执行，已装的包自动跳过）
#
# 内置的实战坑修复：
#   ① rosdep 索引走中科大镜像（raw.githubusercontent.com 国内连不上）
#   ② python3 强制用系统 3.10（容器默认 3.11，与 ROS Humble 冲突，
#      会引发 _rclpy_pybind11 / catkin_pkg 找不到）
#   ③ 激光 gpu_ray → ray（无头环境 GPU 渲染被禁，gpu_ray 发不出 /scan）
#   ④ 清理旧 build 缓存（旧缓存会记住错误的 python 路径）
# ============================================================================

set -e

echo "=================================================="
echo " [1/6] 安装 apt 包（已装的自动跳过）"
echo "=================================================="
sudo apt update
sudo apt install -y \
  build-essential git cmake curl wget \
  python3-pip python3-argcomplete \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-catkin-pkg python3-empy \
  ros-humble-xacro ros-humble-urdf ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-rviz2 \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins \
  ros-humble-nav2-simple-commander ros-humble-slam-toolbox \
  gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
  socat python3-serial

echo "=================================================="
echo " [2/6] rosdep 初始化 + 国内换源"
echo "=================================================="
sudo rosdep init 2>/dev/null || echo "  rosdep 已 init 过，跳过"
sudo rosdep fix-permissions 2>/dev/null || true

# ★ 换源：rosdep 的 yaml 清单（20-default.list）默认指向 raw.githubusercontent.com，
#   实例重置后又会变回默认。必须换成清华 github-raw 镜像，否则 rosdep update 报
#   "Connection reset by peer"（Errno 104）。
#   注意：这跟 ROSDISTRO_INDEX_URL（管 index-v4.yaml）是两码事，两个都要配。
sudo sed -i 's|raw.githubusercontent.com/ros/rosdistro/master|mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master|g' \
  /etc/ros/rosdep/sources.list.d/20-default.list

echo "=================================================="
echo " [3/6] 环境变量持久化（写入 ~/.bashrc）"
echo "=================================================="
# ★ 魔搭 Terminal 读 ~/.bashrc 时会在开头「非交互保护」处提前 return，
#   追加到文件末尾的配置不生效（正是"新终端又变回 3.11、ros2 消失"的原因）。
#   所以：抽到独立文件 ~/.ros_env，再塞到 bashrc 第一行，绕开 return。
cat > ~/.ros_env << 'EOF'
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
EOF

# 注入 ~/.bashrc 第一行
grep -qF 'source ~/.ros_env' ~/.bashrc || sed -i '1i source ~/.ros_env' ~/.bashrc

# ~/.profile 兜底（登录 shell 会读）
grep -qF 'source ~/.ros_env' ~/.profile 2>/dev/null || echo 'source ~/.ros_env' >> ~/.profile

# 当前 shell 立即生效
source ~/.ros_env

echo "=================================================="
echo " [4/6] rosdep update（走中科大索引）"
echo "=================================================="
rosdep update || echo "  [!] rosdep update 网络抖动失败，不影响主线，稍后单独重跑: rosdep update"

echo "=================================================="
echo " [5/6] 修复无头环境激光传感器（gpu_ray → ray，幂等）"
echo "=================================================="
GAZEBO_XACRO=/mnt/workspace/src/navbot_description/urdf/navbot.gazebo.xacro
if [ -f "$GAZEBO_XACRO" ] && grep -q 'gpu_ray' "$GAZEBO_XACRO"; then
  sed -i 's/type="gpu_ray"/type="ray"/' "$GAZEBO_XACRO"
  sed -i 's#<visualize>true</visualize>#<visualize>false</visualize>#' "$GAZEBO_XACRO"
  echo "  已修复：gpu_ray → ray"
else
  echo "  已是 ray 或文件不存在，跳过"
fi

echo "=================================================="
echo " [6/6] 清理旧缓存并编译工作空间"
echo "=================================================="
cd /mnt/workspace
# ★ build/install/log 是上一实例的编译产物，其中的 CMake 缓存记着错误的
#   python 路径（/usr/local/bin/python3），必须清掉，否则 catkin_pkg 报错
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

echo ""
echo "=================================================="
echo " ✅ 环境重建完成"
echo "=================================================="
echo " 快速自检（逐条跑）："
echo "   ros2 pkg list | wc -l                      # 应 300+"
echo "   gazebo --version                           # 应 11.x"
echo "   python3 --version                          # 应 3.10.x（关键！）"
echo "   python3 -c 'import serial, catkin_pkg'     # 应无报错"
echo "   ros2 launch navbot_description display_navbot.launch.py use_rviz:=false"
echo "   （另开终端）ros2 topic hz /odom            # 起仿真后应约 50Hz"
