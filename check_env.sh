#!/usr/bin/env bash
# ============================================================================
# check_env.sh —— navbot 环境体检（3 秒出结论，不改任何东西）
# ============================================================================
# 为什么需要它：魔搭免费实例每次重置，环境层（apt 包 / /opt/ros）都会被清空，
#   而 /mnt/workspace 是持久 NAS（源码不丢）。于是同一个问题会反复出现：
#     · ros2: 未找到命令        ← 环境层没了
#     · Could not find "ament_lint_auto"  ← 装包清单漏了
#     · NO_PUBKEY F42ED6FBAB17C654        ← 密钥权限/源
# 本脚本一次告诉你"现在到底缺什么、该跑哪条命令"，而不是让你重读几百行日志。
#
# 用法：
#   bash check_env.sh          # 体检 + 给出修复建议（只读，不改任何东西）
#   bash check_env.sh --fix    # 体检后直接按缺什么补什么（会装包/编译）
#
# 退出码：0 = 一切正常；1 = 有问题（看报告里的建议）
# ============================================================================

set -u

FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

WORKDIR="/mnt/workspace"
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROSLIST="/etc/apt/sources.list.d/ros2.list"
FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# 记录需要修什么，--fix 时按需执行（而不是盲目重跑 setup_env.sh）
NEED_PKGS=""
NEED_SRC=0
NEED_KEY_PERM=0
NEED_BASHRC=0
NEED_ROS2CLI=0   # 缺 ros2cli（/opt/ros 在但没有 ros2 命令）
NEED_WS_SOURCE=0 # 编译产物在但 overlay 没被 source
# 唯一【脚本无法代劳】的项：要让环境变量在用户当前那个 shell 生效，
#   只能由用户自己敲 source —— 子shell 的 export 影响不到父shell。
NEED_SOURCE_NOW=0

ISSUES=0
say()  { printf '%s\n' "$*"; }
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; ISSUES=$((ISSUES+1)); }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
tip()  { printf '      → %s\n' "$*"; }

say "=================================================="
say " navbot 环境体检"
say "=================================================="

# ---- 0. 环境层是否被重置 ----
say ""
say "【0】环境层状态"
if [ -d /opt/ros/humble ]; then
  # 光看目录不够：v7 实测踩过「/opt/ros/humble 存在、setup.bash 也在，
  #   但 ros2 命令一直报未找到」—— 因为 ros2cli 这个包压根没装（它不是任何
  #   已装包的依赖）。所以这里额外查【ros2 可执行文件】是否真的存在。
  if [ -x /opt/ros/humble/bin/ros2 ]; then
    ok "ROS2 Humble 已安装，且 ros2 可执行文件存在"
    ROS_ONLINE=1
  else
    bad "ROS2 目录在，但【ros2 命令不存在】⇒ 缺 ros-humble-ros2cli 包"
    NEED_ROS2CLI=1
    ROS_ONLINE=1        # ROS 本体在，只是 CLI 缺
  fi
else
  bad "ROS2 未安装 —— 环境层被重置了（魔搭实例每次重置都会这样，属正常）"
  ROS_ONLINE=0
  NEED_SRC=1
  tip "跑完整重建：cd $WORKDIR && bash setup_env.sh"
fi

# 实例重启会导致 hostname 变（dsw-xxxx），用它一眼看出是不是刚重置
HOST=$(hostname 2>/dev/null || echo "?")
say "      当前实例：$HOST"

# ---- 1. ros2 命令能不能用 ----
say ""
say "【1】ros2 命令"
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 在 PATH 里（$(command -v ros2)）"
elif [ "$ROS_ONLINE" -eq 0 ]; then
  bad "ros2 不在 PATH —— 因为 ROS 根本没装（不是 PATH 问题，先跑 setup_env.sh）"
elif [ "${NEED_ROS2CLI:-0}" -eq 1 ]; then
  # 区分开：这次不是"变量没生效"，而是【包没装】—— source 救不了
  bad "ros2 找不到 —— 是【缺 ros-humble-ros2cli 包】，不是环境变量问题"
  say   "    ┌───────────────────────────────────────────────────┐"
  say   "    │ 修复：                                            │"
  say   "    │   sudo apt install -y \\"
  say   "    │     ros-humble-ros2cli \\"
  say   "    │     ros-humble-ros2cli-common-extensions           │"
  say   "    └───────────────────────────────────────────────────┘"
  say   "    （装完 source ~/.ros_env 即可；source 本身救不了缺包）"
else
  # 这是唯一【脚本代劳不了】的问题：要让环境变量在用户当前那个 shell 生效，
  #   只能由用户自己敲 source —— 子 shell 的 export 影响不到父 shell。
  #   所以这里只登记「需要用户手动做」，不假装能自动修。
  NEED_SOURCE_NOW=1
  # 先自查配置文件本身对不对，把能查的先查清
  if [ -f ~/.ros_env ] && head -1 ~/.bashrc 2>/dev/null | grep -q 'source ~/.ros_env'; then
    bad "ros2 不在 PATH，但配置文件【都是对的】⇒ 只是当前这个终端没加载过"
    say "    ┌─────────────────────────────────────────┐"
    say "    │ 立刻可用：在当前终端敲 source ~/.ros_env │"
    say "    └─────────────────────────────────────────┘"
    say "    （配置是对的，所以【新开一个终端】也会自动有 ros2，"
    say "只有你当前这个旧终端需要 source 一次）"
  else
    bad "ros2 不在 PATH，且配置也有问题 ⇒ 需要先修配置再 source"
    tip "修复配置：bash setup_env.sh（它会写 ~/.ros_env 并注入 bashrc）"
    tip "然后在当前终端敲：source ~/.ros_env"
  fi
fi


# ---- 2. 环境变量持久化 ----
say ""
say "【2】环境变量持久化"
if [ -f ~/.ros_env ]; then
  ok "~/.ros_env 存在"
  if head -1 ~/.bashrc 2>/dev/null | grep -q 'source ~/.ros_env'; then
    ok "bashrc 第一行已注入 source ~/.ros_env"
  else
    warn "bashrc 第一行【没有】source ~/.ros_env —— 新终端会变回原样"
    NEED_BASHRC=1
    tip "修复：sed -i '1i source ~/.ros_env' ~/.bashrc"
  fi
else
  warn "~/.ros_env 不存在（setup_env.sh 跑过就会有）"
  tip "修复：跑 setup_env.sh，或手动创建"
fi

# ---- 3. apt 源与密钥 ----
say ""
say "【3】ROS2 apt 源与密钥"
if [ -f "$ROSLIST" ]; then
  ok "ros2.list 存在"
  if grep -q 'mirrors\.\|packages.ros.org' "$ROSLIST" 2>/dev/null; then
    ok "源指向：$(grep -oE 'https?://[^ ]+' "$ROSLIST" | head -1)"
  else
    bad "ros2.list 内容异常：$(cat "$ROSLIST")"
  fi
else
  bad "没有 $ROSLIST —— ROS2 源没配，apt 找不到任何 ros-humble-*"
  NEED_SRC=1
  tip "修复：跑 setup_env.sh（它会幂等配源）"
fi

if [ -s "$KEYRING" ]; then
  PERM=$(stat -c '%a' "$KEYRING" 2>/dev/null || echo "?")
  OTHER=$(printf '%s' "$PERM" | tail -c 1)
  if [ "$OTHER" = "0" ]; then
    bad "密钥权限 $PERM ⇒ _apt 用户读不到 ⇒ apt 会报 NO_PUBKEY"
    NEED_KEY_PERM=1
    tip "修复：sudo chmod 644 $KEYRING"
  else
    ok "密钥权限 $PERM（_apt 可读）"
  fi
  if gpg --show-keys "$KEYRING" 2>/dev/null | grep -q "$FPR"; then
    ok "密钥指纹正确"
  else
    bad "密钥指纹不对/读不出"
    NEED_SRC=1
    tip "修复：sudo rm -f $KEYRING && sudo bash $WORKDIR/setup_env.sh"
  fi
else
  bad "密钥文件不存在：$KEYRING"
  tip "修复：跑 setup_env.sh"
fi

# ---- 4. 关键包（对应 CMakeLists 里的 find_package）----
say ""
say "【4】编译依赖（CMakeLists.txt 里 find_package 找的那些）"
NEED=""
NEED_FULL=""
for p in ros-humble-ament-cmake ros-humble-ament-lint-auto ros-humble-urdf \
         ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
         ros-humble-nav2-bringup ros-humble-slam-toolbox gazebo; do
  if ! dpkg -s "$p" >/dev/null 2>&1; then
    NEED="$NEED ${p#ros-humble-}"      # 短名，仅用于展示（更易读）
    NEED_FULL="$NEED_FULL $p"          # ★ 完整包名，用于 --fix 安装
  fi
done
if [ -z "$NEED_FULL" ]; then
  ok "9 个关键包齐全（ament_lint_auto 在列 —— 缺它编译必挂）"
else
  bad "缺包：$NEED"
  # 登记【完整包名】而非短名：apt install -y ament-cmake 在 Ubuntu 官方源里
  #   【不存在】这个名字，只有 ros-humble-ament-cmake（ROS 源里）。
  #   用短名会导致 --fix 报"Unable to locate package"。（gazebo 是例外，它在 Ubuntu 源）
  NEED_PKGS="$NEED_PKGS$NEED_FULL "
  tip "修复：sudo apt install -y$NEED_FULL"
fi

# ---- 4.5工作空间 overlay 是否已source（v8 补的盲区）----
#   症状：ros2 命令有了、包也齐，但
#     $ ros2 launch navbot_bringup sim_navbot.launch.py
#     Package 'navbot_bringup' not found: searching: ['/opt/ros/humble']
#   原因：install/setup.bash 没被source。setup_env.sh [6/6] 里那句
#     `source install/setup.bash` 只在【那个脚本进程内】生效，退出就没了。
#     新终端必须自己 source 一次。
#   为什么之前没查出来：前面几项查的都是【文件是否存在】（/opt/ros、install/ 都在），
#     而 overlay 加载是【运行时状态】—— 文件在 ≠ 路径被注册进 AMENT_PREFIX_PATH。
say ""
say "【4.5】工作空间 overlay（★ 决定 ros2 launch 能不能找到你项目）"
if [ ! -f "$WORKDIR/install/setup.bash" ]; then
  warn "install/setup.bash 不存在 ⇒ 没编译过，先跑 setup_env.sh"
elif echo "${AMENT_PREFIX_PATH:-}" | grep -q "$WORKDIR/install"; then
  ok "overlay 已加载（AMENT_PREFIX_PATH 含 $WORKDIR/install）"
elif echo "${COLCON_PREFIX_PATH:-}" | grep -q "$WORKDIR/install"; then
  ok "overlay 已加载（COLCON_PREFIX_PATH 含 $WORKDIR/install）"
else
  bad "★ 编译产物在，但 overlay【没被 source】⇒ ros2 launch 找不到 navbot_*"
  say "    ┌────────────────────────────────────────────────────┐"
  say "    │ 修复：在当前终端敲                                │"
  say "    │   source $WORKDIR/install/setup.bash           │"
  say "    └────────────────────────────────────────────────────┘"
  say "    （和 ros2 命令同理：子 shell 的 source 改不了你的终端，"
  say "      这一步只能你手动做；新终端若没自动加载，说明还没写进 ~/.ros_env）"
  NEED_WS_SOURCE=1
fi

# ---- 5. python3 版本（ROS Humble 只支持 3.8~3.10）----
say ""
say "【5】python3 版本"
if [ ! -x /usr/bin/python3 ]; then
  warn "/usr/bin/python3 不存在（Ubuntu 22.04 自带 3.10，说明实例底包异常）"
  tip "修复：重置实例后跑 setup_env.sh"
else
  PYV=$(/usr/bin/python3 -c 'import sys; print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
  if [ "$PYV" = "3.10" ]; then
    ok "/usr/bin/python3 = 3.10（符合 Humble 要求）"
  elif [ "$ROS_ONLINE" -eq 1 ]; then
    bad "/usr/bin/python3 = $PYV ⇒ 与 ROS Humble 不兼容（需 3.10）"
    tip "修复：export PATH=/usr/bin:\$PATH（见 ~/.ros_env）"
  else
    # ROS 还没装时不报为问题，但要把实际值打出来，便于发现底包异常
    say "    /usr/bin/python3 = $PYV（ROS 未装，装完再看这条是否还合适）"
  fi
fi

# ---- 6. 工作空间编译产物 ----
say ""
say "【6】工作空间编译状态"
if [ ! -d "$WORKDIR" ]; then
  bad "$WORKDIR 不存在（源码在持久 NAS 上，正常应该一直在）"
  tip "若是新建实例，需先 git clone 项目"
elif [ ! -d "$WORKDIR/install" ]; then
  if [ "$ROS_ONLINE" -eq 0 ]; then
    warn "install/ 不存在，但 ROS 也没装 ⇒ 先跑 setup_env.sh（它会一并编译）"
  else
    bad "install/ 不存在 ⇒ 还没编译过"
    tip "修复：cd $WORKDIR && colcon build --symlink-install"
  fi
else
  ok "install/ 存在"
  for p in navbot_bridge navbot_bringup navbot_description; do
    if [ -d "$WORKDIR/install/$p" ]; then
      ok "  $p 已编译"
    else
      bad "  $p 没找到（编译可能没完成）"
      tip "修复：cd $WORKDIR && colcon build --symlink-install"
    fi
  done
fi

# ---- 7. GPU 激光（无头环境必须 gpu_ray → ray）----
say ""
say "【7】无头环境激光适配"
XACRO="$WORKDIR/src/navbot_description/urdf/navbot.gazebo.xacro"
if [ -f "$XACRO" ]; then
  if grep -q 'type="gpu_ray"' "$XACRO"; then
    bad "xacro 里还是 gpu_ray ⇒ 无头环境发不出 /scan"
    NEED_SRC=1
    tip "修复：跑 setup_env.sh（会改成 ray）"
  else
    ok "xacro 已是 ray（无头可用）"
  fi
else
  warn "找不到 $XACRO"
fi

# ---- 结论 ----
say ""
say "=================================================="
if [ "$ISSUES" -eq 0 ]; then
  say " ✅ 环境完好，可以直接干活"
  say "=================================================="
  say ""
  say "   bash check_sim.sh                 # 验通仿真层"
  say "   bash record_navigation.sh square  # 录方形导航 GIF"
  exit 0
fi

say " 检出 $ISSUES 处问题"
say "=================================================="

# ---- --fix：按需修复，不盲目重跑整个 setup_env.sh ----
say ""
if [ "$FIX" -eq 1 ]; then
  # root 下sudo 可能不存在
  if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi
  say "=================================================="
  say " --fix 模式：开始修复"
  say "=================================================="

  # 1) 密钥权限（最轻，秒级）
  if [ "$NEED_KEY_PERM" -eq 1 ]; then
    say ""
    say "[1/4] 修密钥权限..."
    $SUDO chmod 644 "$KEYRING" && ok "已 chmod 644" || bad "chmod 失败"
  fi

  # 2) 补装包（含 ros2cli —— 它不是任何已装包的依赖，漏了就没 ros2 命令）
  if [ "$NEED_ROS2CLI" -eq 1 ]; then
    NEED_PKGS="$NEED_PKGS ros-humble-ros2cli ros-humble-ros2cli-common-extensions"
  fi
  if [ -n "$(echo "$NEED_PKGS" | tr -d ' ')" ]; then
    say ""
    say "[2/4] 补装缺包：$NEED_PKGS"
    # shellcheck disable=SC2086
    if $SUDO apt install -y $NEED_PKGS; then
      ok "包已补齐"
    else
      bad "补装失败 —— 多半是 ROS2 源没配好，先跑 setup_env.sh"
    fi
  fi

  # 3) 需要改配置/源码的（源、密钥、xacro、ros_env）→ 交给 setup_env.sh
  if [ "$NEED_SRC" -eq 1 ] || [ "$NEED_BASHRC" -eq 1 ]; then
    say ""
    say "[3/4] 需要改配置/源码，交给你脚本 setup_env.sh 处理"
    if [ -f "$WORKDIR/setup_env.sh" ]; then
      say "     开始跑（幂等，已装的部分会跳过）..."
      # shellcheck disable=SC1091
      if bash "$WORKDIR/setup_env.sh"; then
        ok "setup_env.sh 跑完"
      else
        bad "setup_env.sh 失败 —— 看上面报错"
      fi
    else
      bad "找不到 $WORKDIR/setup_env.sh，无法自动修"
    fi
  fi


  # ---- 4) 编译 ----
  if [ -d "$WORKDIR" ] && [ ! -d "$WORKDIR/install" ]; then
    say ""
    say "[4/4] 编译工作空间..."
    # shellcheck disable=SC1091
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    ( cd "$WORKDIR" && colcon build --symlink-install ) \
      && ok "编译通过" || bad "编译失败 —— 看 log/latest_build/"
  else
    say ""
    say "[4/4] 无需编译（install/ 已存在）"
  fi

  # ---- 脚本代劳不了的那一项，必须显式说清楚 ----
  #   v1 实测踩坑：报告里明明指出「ros2 不在 PATH」，--fix 却只做了 [4/4]
  #   就说"修复结束"，用户接着敲 ros2 依然报未找到命令 —— 因为：
  #     · check_env.sh 是子shell，它 source ~/.ros_env 只影响它自己
  #     · 用户那个终端是【父 shell】，PATH 不会因子shell 而改变
  #   这是 bash 机制，脚本【无法代劳】，只能明确告诉用户下一步敲什么。
  if [ "$NEED_SOURCE_NOW" -eq 1 ]; then
    say ""
    say "=================================================="
    say " ⚠️ 有一项【我代劳不了】，必须你手动执行"
    say "=================================================="
    say ""
    say "   原因：这个体检脚本是子 shell，它 source ~/.ros_env 只影响它自己，"
    say "         改变不了你当前终端（父 shell）的 PATH。"
    say ""
    say "   请在【当前这个终端】直接敲："
    say ""
    say "     source ~/.ros_env"
    say ""
    say "   然后验证："
    say ""
    say "     ros2 pkg list | wc -l      # 应 211（本项目按需装，非 desktop 全套）"
    say ""
    say "   （配置已经是对的，所以新开一个终端也会自动有 ros2，"
    say "     只有你当前这个旧终端需要 source 一次）"
  fi

  say ""
  say "=================================================="
  say " 修复动作结束。确认结果："
  if [ "$NEED_SOURCE_NOW" -eq 1 ]; then
    say "   ① 先在当前终端敲 source ~/.ros_env（这一步只能你做）"
  fi
  if [ "$NEED_WS_SOURCE" -eq 1 ]; then
    say "   ① 先在当前终端敲 source $WORKDIR/install/setup.bash（overlay，只能你做）"
  fi
  say "   ② 再跑 bash check_env.sh 复检"
  say "=================================================="
  exit 1
fi

say ""
if [ "$ROS_ONLINE" -eq 0 ]; then
  say " ★ 环境层被重置了 —— 魔搭实例重置的正常现象，不是 bug"
  say ""
  say "   一条命令重建（15~25 分钟，幂等可重复跑）："
  say ""
  say "     cd $WORKDIR && bash setup_env.sh"
  say ""
  say "   装完自检：bash check_env.sh"
else
  say " 环境层还在，只是部分环节缺失。可以让脚本自动修："
  say ""
  say "     bash check_env.sh --fix"
  say ""
  say "（或按上面各条 → 提示手动执行）"
fi
exit 1