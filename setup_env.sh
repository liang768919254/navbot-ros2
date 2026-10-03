#!/usr/bin/env bash
# setup_env.sh —— navbot ROS2 Humble 环境一键重建（魔搭免费实例专用）
#
# 魔搭实例重置后，环境层（apt 包、系统配置）会被清空，而 /mnt/workspace 是
# 持久 NAS，源码不丢。本脚本负责重建环境层，15~25 分钟，幂等可重复跑。
#
#   cd /mnt/workspace && bash setup_env.sh
#
# ── 为什么这个脚本必须自己配源 ─────────────────────────────────
# 装 ROS 的第一步是加 ROS2 的 apt 源，而这步一旦漏掉，后面全盘失败：
# apt 只看得到 Ubuntu 官方源，而官方源里 ros-humble-* 一个都没有。
# 实测：ROS2 源 jammy 索引 7858 个包，脚本 18 个依赖全部命中；
#       Ubuntu jammy main+universe 索引 65013 个包，ros-humble-* 命中 0 个。
# 最容易误判的是那三个 python3-* 包（colcon / rosdep / vcstool），
# 它们的名字是对的，只是只存在于 ROS2 源 —— Ubuntu 官方源里 colcon 根本没打进，
# rosdep 只有旧版的 python3-rosdep2，vcstool 只有 python3-vcstools。
#
# 国内可用镜像（InRelease 均 200，索引大小一致）：
#   中科大 → 清华 → 阿里云 → 官方（国内慢，放最后兜底）
# 密钥只走中科大 rosdistro 镜像，raw.githubusercontent.com 国内不通（Errno 104）。
# 密钥指纹 C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654（Open Robotics，rsa4096）。
#
# ── NO_PUBKEY 的坑：密钥权限 600 时 apt 认不出来 ─────────────────
# apt 验签不是以 root 跑 gpg，而是以 _apt 用户执行 gpgv 去读那个文件。
# 读不到 600 的文件等于没装 key，报错是 NO_PUBKEY <keyid>。
# 注意别用 `gpg --show-keys` 自检：那是 root 视角读的，600 也能读，
# 会永远显示「校验通过」而 apt 仍然失败。判断标准只有一个：
#   权限的 other 位可读（644）。
# 同脚本里两条写入分支产物权限还不一样（cp 会保留源文件的 600，
# 重定向创建是 644），所以统一 chmod 644。
#
# ── rosdep 的坑：init 和换源的顺序反了就永远走不到 ────────────
# rosdep init 干的事就是去下载 raw.githubusercontent.com 上的清单，
# 国内连不上 ⇒ init 自己先失败，文件根本下不下来，
# 「先 init 再换源」这个顺序在国内永远等不到第二步。
# 正确做法是绕开 init，直接把清单写成镜像版内容（固定 4 条 yaml URL，
# 不含机器相关信息，写死是安全的，效果与 init 等价）。
# 另外 rosdep 失败不阻塞本项目：colcon build 默认模式不走 rosdep，
# 只解析 package.xml 里的声明，而这些依赖都已用 apt 显式装齐。
#
# ── ros2cli 是漏装过的包（不报错，最难查）─────────────────────
# ros2 可执行文件由 ros-humble-ros2cli 提供，而下面装的这一批包
# 没有一个依赖它，所以它从来都是漏的。漏了不会编译失败、
# 环境检查也全绿，只是敲 ros2 报「未找到命令」。
# 注意 ros-humble-ros-base 也不含 ros2cli（它的 Depends 里只有 ros-workspace），
# 光装 base 解决不了。
#
# ── ament_lint_auto 也是漏装过的包（会编译失败）───────────────
# package.xml 里它是 test_depend，但 CMakeLists.txt 里有
# find_package(ament_lint_auto REQUIRED)，而 colcon build 默认
# BUILD_TESTING=ON，这段会执行 ⇒ 缺包直接报
# Could not find a package configuration file provided by "ament_lint_auto"。
# 这段是 ros2 pkg create 模板自带的，模板新生成的新包也躲不过。
# 只装 ament_lint_auto 不装 ament_lint_common：后者会连带拉七八个
# linter，而我们不跑 colcon test，缺的 linter 会被自动跳过。
#
# ── 三个 python3 版本 / 环境变量相关的坑 ─────────────────────
# · 魔搭容器默认 python3 是 3.11（装在 /usr/local/bin），ROS Humble 只支持
#   3.8~3.10，会让 _rclpy_pybind11 / catkin_pkg 找不到。所以显式校验
#   /usr/bin/python3 必须是 3.10，不对就停下说明原因。
# · 魔搭 Terminal 读 ~/.bashrc 时会在开头的「非交互保护」处提前 return，
#   追加到文件末尾的配置不生效。抽到独立文件 ~/.ros_env 再塞到 bashrc 第一行。
# · ~/.ros_env 里还要 source 工作空间的 setup.bash，否则 ros2 命令有了
#   但 ros2 launch 找不到 navbot_*（报 searching: ['/opt/ros/humble']）。
#   注意 setup_env.sh 里那句 source install/setup.bash 只在脚本进程内生效。
#
# ── 其他内置修复 ────────────────────────────────────────────
# · 激光 gpu_ray → ray：无头环境 GPU 渲染被禁，gpu_ray 发不出 /scan
# · 清理旧 build 缓存（里面的 CMake 缓存记着上一实例的错误 python 路径）
# · root 登录时 sudo 可能不存在，自动降级
#
# ── 两条 bash 陷阱（语法合法、行为错误，bash -n 查不出）────────
# · cmd | tee log 的退出码是 tee 的（恒 0），if 永远走成功分支，
#   要判断真实结果必须 set -o pipefail。末段是 sed/head/cat/wc 同理。
# · `echo"xxx"` 少一个空格在 bash 里语法完全合法（被当成一个命令名），
#   只有运行时才炸。改完脚本跑一下 audit_scripts.sh 查这两类。

set -e

# root 降级：魔搭实例常是 root 直登，容器里可能没有 sudo
if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
else
  SUDO="sudo"
fi

# 国内 ROS2 源候选（按实测国内速度排序，官方放最后兜底）
ROS_MIRRORS=(
  "https://mirrors.ustc.edu.cn/ros2/ubuntu"
  "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
  "https://mirrors.aliyun.com/ros2/ubuntu"
  "http://packages.ros.org/ros2/ubuntu"
)
ROS_KEY_URL="https://mirrors.ustc.edu.cn/rosdistro/ros.key"
ROS_DISTRO="humble"
UBUNTU_CODENAME="jammy"     # Ubuntu 22.04；换 20.04 记得改这里

echo "=================================================="
echo " [0/6] 配置 ROS2 apt 源（v2 新增：之前报「无法定位软件包」的根因）"
echo "=================================================="

# ---- 0.1 密钥 -> /usr/share/keyrings/ros-archive-keyring.gpg ----
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
# 已知指纹（Open Robotics <info@osrfoundation.org>），校验用，防止下到 HTML 错误页
ROS_KEY_FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

if [ -s "$KEYRING" ] && gpg --show-keys "$KEYRING" 2>/dev/null | grep -q "$ROS_KEY_FPR"; then
  # 已存在也要纠权限：_apt 读不到 600 的密钥（原因见文件头）
  #   ⇒ 明明 gpg --show-keys 校验通过，apt update 却报 NO_PUBKEY
  $SUDO chmod 644 "$KEYRING"
  echo "  ✅ ROS2 密钥已就位（指纹校验通过，权限 $(stat -c '%a' "$KEYRING" 2>/dev/null || echo '?'))"
else
  echo "  下载 ROS2 密钥（走中科大 rosdistro 镜像）..."
  TMP_KEY=$(mktemp)
  if curl -fsSL --max-time 30 "$ROS_KEY_URL" -o "$TMP_KEY"; then
    # 镜像可能给二进制 keyring，也可能给 ASCII armored；两种都支持
    # 判据：gpg 能直接 show-keys 就是二进制；不能就 dearmor
    if gpg --show-keys "$TMP_KEY" >/dev/null 2>&1; then
      $SUDO cp "$TMP_KEY" "$KEYRING"
    else
      $SUDO gpg --dearmor < "$TMP_KEY" > "$KEYRING"
    fi
    # 权限必须显式给 644，否则 _apt 读不到（原因见文件头）
    #   mktemp 建的临时文件是 0600，cp 会【保留源文件权限】⇒ keyring 变 600
    #   ⇒ apt 以 _apt 用户跑 gpgv 读不到 ⇒ NO_PUBKEY <keyid>
    #   而 `gpg --dearmor > file` 是重定向创建（umask 022⇒ 644），所以那条分支
    #   不会中招 —— 同一个脚本两条分支行为不一致，极难排查。
    #   修法统一用 install -m 644：一次搞定"写内容 + 定权限"，不依赖 umask。
    $SUDO chmod 644 "$KEYRING"
    if gpg --show-keys "$KEYRING" 2>/dev/null | grep -q "$ROS_KEY_FPR"; then
      # 权限自查：把 _apt 读不到这件事挡在 apt update 之前
      PERM=$(stat -c '%a' "$KEYRING" 2>/dev/null || echo "?")
      if [ "$PERM" != "644" ] && [ "$PERM" != "664" ] && [ "$PERM" != "666" ]; then
        echo "  ❌ 密钥权限是 $PERM，_apt 用户读不到（必须是 644）"
        echo "     手动修：sudo chmod 644 $KEYRING && sudo apt update"
        rm -f "$TMP_KEY"
        exit 1
      fi
      echo "  ✅ 密钥指纹校验通过（权限 $PERM）"
    else
      echo "  ❌ 密钥指纹不对（不是 Open Robotics 官方 key），请检查网络是否被劫持"
      rm -f "$TMP_KEY"
      exit 1
    fi
  else
    echo "  ❌ 密钥下载失败：$ROS_KEY_URL"
    echo "     可手动下载后放好：sudo curl -fsSL $ROS_KEY_URL -o $KEYRING && sudo chmod 644 $KEYRING"
    rm -f "$TMP_KEY"
    exit 1
  fi
  rm -f "$TMP_KEY"
fi

# ---- 0.2 写源（幂等：先删掉旧的再写，避免重复行）----
#用 codename 而不是写死 jammy
CODENAME=$(lsb_release -cs 2>/dev/null || echo "$UBUNTU_CODENAME")
ROSLIST="/etc/apt/sources.list.d/ros2.list"

pick_mirror() {
  local m
  for m in "${ROS_MIRRORS[@]}"; do
    if curl -fsS --max-time 12 -o /dev/null "$m/dists/$CODENAME/InRelease"; then
      echo "$m"; return 0
    fi
    echo "    （$m 不通，试试下一个）" >&2
  done
  return 1
}

echo "  探测可用镜像..."
MIRROR=$(pick_mirror) || {
  echo "  ❌ 所有 ROS2 镜像都不通。检查网络/代理后重试，或手动写 $ROSLIST"
  exit 1
}
echo "  使用镜像：$MIRROR"

$SUDO rm -f "$ROSLIST"
$SUDO tee "$ROSLIST" > /dev/null <<EOF
deb [arch=amd64 signed-by=$KEYRING] $MIRROR $CODENAME main
EOF

# ---- 0.3 universe 组件检查（gazebo / empy / catkin-pkg 在 universe 里）----
if ! grep -rqsE '^Components:.*\buniverse\b' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null \
   && ! grep -rqsE '^\s*deb .*\b(universe)\b' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
  echo "  ⚠️  没检测到 universe 组件（gazebo / python3-empy / python3-catkin-pkg 在里面）"
  echo "     尝试自动启用..."
  if [ -f /etc/apt/sources.list ]; then
    $SUDO sed -i 's/^\(deb .* jammy.*\)$/\1 universe/; s/^\(deb .* jammy-updates.*\)$/\1 universe/' /etc/apt/sources.list || true
  fi
  for f in /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; do
    [ -f "$f" ] || continue
    grep -q 'universe' "$f" || $SUDO sed -i 's/^\(Components:.*\)$/\1 universe/' "$f" 2>/dev/null || true
  done
fi

# ---- 0.4 验签自检：用 apt 内部同款 gpgv 真验一次签（必须在 apt update 之前）----
#   为什么必须加这一步（v2 初版翻车点）：
#     `gpg --show-keys` 只证明"密钥文件里有个对的公钥"，
#     但 apt 真正做的是「以 _apt 用户身份跑 gpgv 读这个文件」——
#     文件权限 600 / 路径不对，都会让前者过、后者挂。
#     v2 初版就是这么翻的：明明打印"✅ 密钥指纹校验通过"，apt 却报 NO_PUBKEY。
#   所以这里用【和apt 完全一样的命令】先试一次，过了才继续。
echo "  验签自检（用 apt 内部同款 gpgv）..."
# 三态区分：把"网络拉不到"和"验签失败"都算成失败的话，
# 网络一抖就会报"密钥坏了"并 exit 1，白白中断安装
#     通过→ 继续
#     失败      → 真的密钥问题，必须停
#     没验成    → 网络问题，降级放行（交给 apt update 出最终结论，不阻塞）
SELF_STATE="未验证"
for m in "$MIRROR" "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu" \
         "https://mirrors.aliyun.com/ros2/ubuntu" "http://packages.ros.org/ros2/ubuntu"; do
  TMP_INR=$(mktemp)
  if curl -fsSL --max-time 25 "$m/dists/$CODENAME/InRelease" -o "$TMP_INR" 2>/dev/null; then
    if gpgv --keyring "$KEYRING" "$TMP_INR" >/dev/null 2>&1; then
      echo "  ✅ 验签通过（$(echo "$m" | cut -d/ -f3)）"
      SELF_STATE="通过"
    else
      echo "  ❌ 验签失败（$(echo "$m" | cut -d/ -f3)）：gpgv 读不到密钥，或签名不匹配"
      echo "     密钥现状：$(stat -c '权限=%a 属主=%U:%G' "$KEYRING" 2>/dev/null || echo '文件不存在')"
      echo "     _apt 用户必须能读它：sudo chmod 644 $KEYRING"
      SELF_STATE="失败"
    fi
    rm -f "$TMP_INR"
    break            # 拿到 InRelease 就有结论了，无论成败
  fi
  rm -f "$TMP_INR"
done

if [ "$SELF_STATE" = "失败" ]; then
  echo ""
  echo "  ❌ 密钥验签失败，停止安装（否则 apt update 必然报 NO_PUBKEY）"
  echo "     手动修复三选一："
  echo "     1) 权限：sudo chmod 644 $KEYRING && sudo apt update"
  echo "     2) 重下：sudo rm -f $KEYRING && sudo curl -fsSL $ROS_KEY_URL -o $KEYRING && sudo chmod 644 $KEYRING"
  echo "     3) 换官方源：把 $ROSLIST 里的镜像换成 http://packages.ros.org/ros2/ubuntu"
  exit 1
elif [ "$SELF_STATE" = "未验证" ]; then
  # 网络原因导致没验成 ≠ 密钥有问题，不该在这里 exit 1 阻塞整个安装
  #   交给下面的 apt update 出最终结论（它的报错才是权威判据）。
  echo "  ⚠️  几个镜像都拉不到 InRelease ⇒ 本项没验成（不代表密钥有问题）"
  echo "     继续往下走，由 apt update 做最终判定；若那里报 NO_PUBKEY 再说"
fi

# ---- 0.5 apt update ----
# 两个必须注意的 bash 陷阱（都实测确认过）：
#     ① `apt update | tee log` 的退出码是【tee 的】，不是 apt 的
#        —— 不加 pipefail 的话 if 永远走"成功"分支，等于这个检查形同虚设。
#     ② apt 的退出码把「网络抖动」和「源不可用」混在一起，set -e 下直接崩、
#        什么都看不清。所以要自己抓日志、分类给处置。
echo "  apt update..."
APT_LOG=$(mktemp)
set -o pipefail                      # 让管道退出码反映 apt 的真实结果
$SUDO apt update 2>&1 | tee "$APT_LOG"
APT_RC=$?
set +o pipefail
if [ "$APT_RC" -eq 0 ]; then
  echo "  ✅ apt update 完成"
else
  if grep -qE 'NO_PUBKEY|没有数字签名|无法验证下列签名' "$APT_LOG"; then
    echo ""
    echo "  ❌ apt update 报签名问题 —— 密钥仍不可用"
    echo "     跑专门诊断：bash diag_ros_apt_key.sh"
    echo "     快速尝试：sudo chmod 644 $KEYRING && sudo apt update"
    rm -f "$APT_LOG"
    exit 1
  fi
  # 纯网络/索引问题：ROS 源没拉到索引就退到官方源再试一次
  if ! grep -q 'mirrors.ustc.edu.cn/ros2' "$APT_LOG"; then
    echo "  ⚠️  ROS2 源没拉到索引，换官方源重试..."
    $SUDO tee "$ROSLIST" > /dev/null <<EOF
deb [arch=amd64 signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu $CODENAME main
EOF
    set -o pipefail
    $SUDO apt update 2>&1 | tee "$APT_LOG"
    APT_RC=$?
    set +o pipefail
    if [ "$APT_RC" -ne 0 ]; then
      echo ""
      echo "  ❌ 官方源也失败。请检查网络/代理后重跑本脚本（本脚本幂等）。"
      echo "     完整日志：$APT_LOG"
      exit 1
    fi
    echo "  ✅ 换官方源后成功"
  else
    echo ""
    echo "  ❌ apt update 失败（看上面报错）。若是网络问题，重跑本脚本即可（幂等）。"
    echo "     完整日志：$APT_LOG"
    exit 1
  fi
fi
rm -f "$APT_LOG"

# ---- 0.6 装之前就先验证 apt 能找到这些包，别等装到一半才报一屏 E: ----
echo "  验证 apt 现在能不能找到 ROS2 包..."
PROBE_PKGS=(ros-humble-xacro ros-humble-rviz2 ros-humble-navigation2
            ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
            python3-rosdep python3-vcstool)
MISSING=""
for p in "${PROBE_PKGS[@]}"; do
  apt-cache show "$p" > /dev/null 2>&1 || MISSING="$MISSING $p"
done
if [ -n "$MISSING" ]; then
  echo "  ❌ 配源后 apt 仍然找不到这些包：$MISSING"
  echo ""
  echo "     排查顺序（逐条手动跑，第一条能解决就别看后面）："
  echo "     1) 源文件内容是否正确：cat $ROSLIST"
  echo "     2) codename 对不对：本机是 '$CODENAME'，ROS2 Humble 只支持 jammy/focal"
  echo "     3) 索引有没有真下来：ls -lh /var/lib/apt/lists/ | grep ros2"
  echo "     4) 换个镜像重试：把上面 MIRROR 那行换成 tuna 或 aliyun 再 apt update"
  echo "     5) 密钥问题：apt update 时若报 NO_PUBKEY，"
  echo "        重新执行本步骤（[0/6]）即可，它会覆盖写 keyring"
  exit 1
fi
echo "  ✅ 全部 7 个探针包都能被 apt 找到了"

echo "=================================================="
echo " [1/6] 基础构建工具（Ubuntu 官方源，失败通常是网络/universe 问题）"
echo "=================================================="
$SUDO apt install -y \
  build-essential git cmake curl wget \
  python3-pip python3-argcomplete \
  python3-empy python3-catkin-pkg \
  socat python3-serial

echo "=================================================="
echo " [2/6] ROS2 Humble + Gazebo + Nav2（已装的自动跳过）"
echo "=================================================="
$SUDO apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-ros2cli ros-humble-ros2cli-common-extensions \
  ros-humble-xacro ros-humble-urdf ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-rviz2 \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins \
  ros-humble-nav2-simple-commander ros-humble-slam-toolbox \
  gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
  ros-humble-ament-lint-auto

# ros2cli + ros2cli-common-extensions：缺它就没有 ros2 命令（原因见文件头）
#   实测症状：环境全部装好、colcon build 通过、setup.bash 也在，
#     但敲 ros2 一直报「未找到命令」，source ~/.ros_env 也救不了。
#   根因：ros2 可执行文件由 ros-humble-ros2cli 提供，而 v1~v6 装的那15 个包
#     【没有一个依赖 ros2cli】（逐个查过 Depends 确认），所以它从没被装上。
#     这类漏装最难靠日志发现，因为所有已装的包都正常。
#   为什么装两个：
#     · ros2cli                  → 提供 /opt/ros/humble/bin/ros2 这个入口
#     · ros2cli-common-extensions→ 带全部子命令。实测其Depends 含：
#         ros2action / ros2component / ros2doctor / ros2interface / ros2launch /
#         ros2lifecycle / ros2multicast / ros2node / ros2param / ros2pkg /
#         ros2plugin / ros2run / ros2service / ros2topic / sros2
#       本项目脚本用到的 6 个（run/topic/node/service/action/lifecycle）全在里，
#       顺带还得了 ros2 launch（启动 .launch.py 必需）。
#   注意 ros-humble-ros-base【不含】 ros2cli（实测它的 Depends 里没有），
#     所以光装 ros-base 解决不了这个问题 —— 别被"装个 base 就够了"带偏。

# ament_lint_auto 是 colcon build 硬性要求的（原因见文件头）
#   本项目 navbot_description / navbot_bringup 两个 ament_cmake 包的
#   CMakeLists.txt 里都有这段（ros2 pkg create 模板自带）：
#       if(BUILD_TESTING)
#         find_package(ament_lint_auto REQUIRED)
#         ament_lint_auto_find_test_dependencies()
#       endif()
#   而 colcon build 默认就是 BUILD_TESTING=ON ⇒ 这段会执行 ⇒ 缺包直接报：
#       Could not find a package configuration file provided by "ament_lint_auto"
#   ⇒ 装包时漏了它，编译必挂。已装则 apt 自动跳过。
#
#   为什么只装它、不装 ament_lint_common：
#     · ament_lint_auto 自身依赖只有 ament_cmake-core / ament-cmake-test / ros-workspace
#     · ament_lint_common 会连带拉进 copyright/cppcheck/cpplint/flake8/pep257/
#       uncrustify/xmllint 七八个 linter（体积不小），而我们不跑 colcon test
#     · 缺的 linter 会被 find_test_dependencies 自动跳过，不影响 build
#   ⇒ 够用即可，要跑 colcon test 时再补 ament_lint_common。

echo "=================================================="
echo " [3/6] 环境变量持久化（写入 ~/.ros_env + bashrc 首行）"
echo "=================================================="
# 魔搭 Terminal 读 ~/.bashrc 时会在开头「非交互保护」处提前 return，
#   追加到文件末尾的配置不生效（正是"新终端又变回 3.11、ros2 消失"的原因）。
#   所以：抽到独立文件 ~/.ros_env，再塞到 bashrc 第一行，绕开 return。
cat > ~/.ros_env << 'EOF'
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
# 把工作空间 overlay 也一起加载。
#   为什么必须有：ros2 命令来自 /opt/ros/humble（上一行已解决），
#   但 `ros2 launch navbot_bringup ...` 要在 AMENT_PREFIX_PATH 里
#   找到 /mnt/workspace/install/navbot_bringup —— 那是【编译产物】路径，
#   不在这里就报Package 'navbot_bringup' not found。
#   加了 -f 判断：overlay 还没编译时这一行静默跳过，不报错。
[ -f /mnt/workspace/install/setup.bash ] && source /mnt/workspace/install/setup.bash
EOF

# 注入 ~/.bashrc 第一行
grep -qF 'source ~/.ros_env' ~/.bashrc || sed -i '1i source ~/.ros_env' ~/.bashrc

# ~/.profile 兜底（登录 shell 会读）
grep -qF 'source ~/.ros_env' ~/.profile 2>/dev/null || echo 'source ~/.ros_env' >> ~/.profile

# python3 必须是 3.10：ROS Humble 官方支持 3.8~3.10，3.11 会让
#   _rclpy_pybind11 / catkin_pkg 找不到。这里显式校验，别等colcon 报一屏栈。
PYV=$(/usr/bin/python3 -c 'import sys; print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
if [ "$PYV" != "3.10" ]; then
  echo "  ❌ /usr/bin/python3 是 $PYV，ROS Humble 需要 3.10"
  echo "     本机 python3 在哪：which -a python3 | head -5"
  echo "     若 /usr/bin/python3 不存在或版本不对，说明实例底包异常，"
  echo "     建议重置实例后重跑本脚本（别用 pip 强行改 python 版本，会弄坏系统）"
  exit 1
fi
echo "  ✅ /usr/bin/python3 = $PYV（符合 Humble 要求）"

# 当前 shell 立即生效
# shellcheck disable=SC1091
source ~/.ros_env

echo "=================================================="
echo " [4/6] rosdep 国内源 + update"
echo "=================================================="
# 顺序【先 init 再换源】从根上就是错的（原因见文件头）
#
#   为什么错（v6 实测踩坑，Errno 104 Connection reset by peer）：
#     `rosdep init` 干的事就是【去下载 raw.githubusercontent.com 上的
#     20-default.list】。国内连不上那个域名 ⇒ init 自己就先失败了，
#     文件根本下载不下来 ⇒ 脚本"再换源"那一步永远等不到执行。
#     ⇒ 换源必须【在 init 之前】完成，或者干脆【绕开 init】。
#
#   ⇒ 现在的正确做法（两步，不再依赖 rosdep init 的联网）：
#     1) 直接把 20-default.list 写死成【镜像版】内容（只8 行，纯 yaml URL）
#        —— 绕开 init 那次对 raw.githubusercontent 的下载
#     2) rosdep update 拉索引 —— 这个走 ROSDISTRO_INDEX_URL（中科大），能通
#
#   为什么可以直接写死：20-default.list 只是一份【rosdep 规则库的索引清单】，
#     内容是固定的 4 条 yaml URL（base/python/ruby/osx-homebrew），
#     不含任何机器相关信息，换台机器内容一样 ⇒ 值得直接写进脚本。
ROSDEP_DIR="/etc/ros/rosdep/sources.list.d"
ROSDEP_LIST="$ROSDEP_DIR/20-default.list"
# 清华 github-raw 镜像（实测 200）；raw.githubusercontent.com 国内不通
ROSDEP_MIRROR_BASE="https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep"

if [ -f "$ROSDEP_LIST" ] && grep -q 'mirrors.tuna\|mirrors.ustc' "$ROSDEP_LIST" 2>/dev/null; then
  echo "  ✅ rosdep 源已指向国内镜像，跳过"
else
  echo "  写入 rosdep 源清单（镜像版，绕开 rosdep init 的境外下载）..."
  $SUDO mkdir -p "$ROSDEP_DIR"
  $SUDO tee "$ROSDEP_LIST" > /dev/null <<EOF
# os-specific listings first
yaml ${ROSDEP_MIRROR_BASE}/osx-homebrew.yaml osx

# generic
yaml ${ROSDEP_MIRROR_BASE}/base.yaml
yaml ${ROSDEP_MIRROR_BASE}/python.yaml
yaml ${ROSDEP_MIRROR_BASE}/ruby.yaml

# newer distributions (Groovy, Hydro, ...) must not be listed anymore,
# they are being fetched from the rosdistro index.yaml instead
EOF
  echo "  ✅ 已写入 $ROSDEP_LIST（清华 github-raw 镜像）"
fi

$SUDO rosdep fix-permissions 2>/dev/null || true

# 注意：这里【不再调 rosdep init】—— 它会去下载 raw.githubusercontent 而失败。
# 上面直接写文件，效果与 init 相同（init 本质就是往这个路径写这份清单）。
echo "  跳过 rosdep init（它只会去下载连不上的 raw.githubusercontent.com）"

# rosdep update：拉的是规则库索引，走 ROSDISTRO_INDEX_URL（中科大），能通
# 管道退出码陷阱：`rosdep update | sed` 的退出码是 sed 的（恒 0），必须开 pipefail
ROSDEP_LOG=$(mktemp)
set -o pipefail
rosdep update 2>&1 | tee "$ROSDEP_LOG" | sed 's/^/  /'
ROSDEP_RC=$?
set +o pipefail
if [ "$ROSDEP_RC" -eq 0 ]; then
  echo "  ✅ rosdep update 完成"
else
  echo ""
  echo "  [!] rosdep update 没成功 —— 【不影响本项目主线，别在这里卡住】"
  echo "      原因已存：$ROSDEP_LOG"
  echo "      · 本项目 package.xml 的依赖都已用 apt 显式装齐"
  echo "      · colcon build 默认模式【不走 rosdep】（它只解析 package.xml 声明）"
  echo "      · rosdep 只在你想「按声明自动补装依赖」时才需要"
  echo "      需要时手动重跑：rosdep update"
fi
rm -f "$ROSDEP_LOG"

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
# build/install/log 是上一实例的编译产物，其中的 CMake 缓存记着错误的
#   python 路径（/usr/local/bin/python3），必须清掉，否则 catkin_pkg 报错
rm -rf build install log

# 编译前预检：把"必然导致编译失败的缺包"在这里查出来，
# 而不是等 colcon 报一屏 CMake Error
MISSING_BUILD=""
for p in ros-humble-ament-cmake ros-humble-ament-lint-auto ros-humble-urdf \
         ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
         ros-humble-nav2-bringup ros-humble-slam-toolbox; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING_BUILD="$MISSING_BUILD $p"
done
if [ -n "$MISSING_BUILD" ]; then
  echo ""
  echo "  ⚠️  编译依赖还没装齐：$MISSING_BUILD"
  echo "     （colcon build 会报 Could not find a package configuration file provided by ...）"
  echo "     正在补装..."
  # shellcheck disable=SC2086
  $SUDO apt install -y $MISSING_BUILD || {
    echo "  ❌ 补装失败。请手动执行：sudo apt install -y $MISSING_BUILD"
    exit 1
  }
  echo "  ✅ 补装完成"
fi

# colcon build：不让 set -e 裸崩，失败时给出可定位的诊断
echo ""
echo "  开始 colcon build（首次约 1~3 分钟）..."
# 同一个管道陷阱：colcon build | tee 的退出码是 tee 的，必须开 pipefail
set -o pipefail
colcon build --symlink-install 2>&1 | tee /tmp/setup_build.log
BUILD_RC=$?
set +o pipefail
if [ "$BUILD_RC" -eq 0 ]; then
  echo "  ✅ 编译通过"
else
  echo ""
  echo "  ❌ colcon build 失败。按包定位："
  grep -E '^(Failed|Aborted|Finished) ' /tmp/setup_build.log 2>/dev/null | sed 's/^/     /' || true
  echo ""
  echo "     最常见的两种（按概率排序）："
  echo "     1) Could not find a package configuration file provided by \"ament_lint_auto\""
  echo "        → apt install -y ros-humble-ament-lint-auto"
  echo "     2) 缺某个 find_package 里的包 → 看 CMakeLists.txt 第 9~16 行"
  echo "        里的包名，装对应的 ros-humble-<包名>"
  echo ""
  echo "     完整日志：/tmp/setup_build.log"
  echo "     查具体报错：grep -A5 'CMake Error' /tmp/setup_build.log"
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

# 编译结果自检：三个包都要在 install/ 下能找到
echo ""
echo "  编译产物自检："
BUILD_OK=1
for p in navbot_bridge navbot_bringup navbot_description; do
  if [ -d "install/$p" ] || ls install/*/share/$p >/dev/null 2>&1; then
    echo "    ✅ $p"
  else
    echo "    ❌ $p 没找到（编译可能没真正完成）"
    BUILD_OK=0
  fi
done
if [ "$BUILD_OK" -eq 0 ]; then
  echo ""
  echo "  ⚠️  有包没产出，检查 /tmp/setup_build.log"
fi

echo ""
echo "=================================================="
echo " ✅ 环境重建完成"
echo "=================================================="
echo "  快速自检（逐条跑）："
echo "   source ~/.ros_env"
echo "   ros2 pkg list | wc -l                      # 应 211（本项目按需装，非 desktop 全套）"
echo "   gazebo --version                           # 应 11.x"
echo "   python3 --version                          # 应 3.10.x（关键！）"
echo "   python3 -c 'import serial, catkin_pkg'# 应无报错"
echo "   ros2 launch navbot_description display_navbot.launch.py use_rviz:=false"
echo "   （另开终端）ros2 topic hz /odom            # 起仿真后应约 50Hz"
echo ""
echo "  下一步："
echo "   bash check_sim.sh                # 先单独验通仿真层"
echo "   bash record_navigation.sh square # 录方形导航 GIF"
