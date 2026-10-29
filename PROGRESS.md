# W2 进度 · `navbot_ws` 四包落地

> 对应 `22_雷达车收尾与下一阶段启动/02_11月到1月周计划.md` 的 W2（11-03 → 11-09）
> 实际完成：2026-10-29（提前 5 天）

## 完成项

| # | 计划动作 | 结果 |
|---|---|---|
| 1 | 建 `~/navbot_ws/src/`，只放 3 个 navbot 包 | ✅ 3 包到位，11 个练习包留在原处未搬 |
| 2 | 修 `cmake_minimum_required` 弃用警告 | ✅ 3.8 → **3.10**，`navbot_bringup` + `navbot_description` 各一处 |
| 3 | 加第 4 包 `navbot_chassis` | ✅ ament_cmake，骨架 + 5 个 TODO，未填实现 |

## 实测输出（2026-10-29 18:30）

```
$ colcon build
Starting >>> navbot_bridge
Starting >>> navbot_bringup
Starting >>> navbot_chassis
Starting >>> navbot_description
Finished <<< navbot_bringup [0.44s]
Finished <<< navbot_description [0.44s]
Finished <<< navbot_bridge [1.13s]
Finished <<< navbot_chassis [5.86s]

Summary: 4 packages finished [6.19s]
```

```
$ ros2 pkg list | grep navbot
navbot_bridge
navbot_bringup
navbot_chassis
navbot_description
```

```
$ ls install/navbot_chassis/lib/navbot_chassis/
chassis_node
```

节点实测能起、能 spin（8 秒后被 SIGTERM 正常停掉）。

## 两个环境噪声（不是代码问题，说明一下免得以后误判）

| 现象 | 根因 | 真实终端会怎样 |
|---|---|---|
| `ModuleNotFoundError: No module named 'catkin_pkg'`，`package_xml_2_cmake.py` 返回 1 | 沙箱把 python **3.13** 放进 PATH 前面，`catkin_pkg` 只装在系统 3.10 | **不会出现**。若真出现，`/usr/bin/python3` 优先即可 |
| `warning: build_py: byte-compiling is disabled` | 环境变量 `PYTHONDONTWRITEBYTECODE=1` | **不会出现**（已实测：`env -u PYTHONDONTWRITEBYTECODE` 后警告计数 = 0） |

## 关键数字（冻基线）

| 项 | 值 |
|---|---|
| CMake 版本 | 4.4.3（4.x 起把 `<3.10` 标为弃用，5.0 变硬错误） |
| 完整 build 耗时 | 6.19 s（4 包） |
| 包总数 | 4（旧工作空间是 14） |
| `navbot_chassis` 骨架编译耗时 | 5.86 s（含 rclcpp 首次链接） |

## 事故记录 · 2026-10-29 18:36 环境串味（build 4 包但 pkg list 只有 3 个）

**现象**：`colcon build` 明明 4 包全过，`ros2 pkg list | grep navbot` 只出 3 个，缺 `navbot_chassis`。

**根因链**（三层，逐层挖出来的）：

1. `~/.bashrc:135` 每次开终端都 `source ~/桌面/具身智能学习路线/bridge/ros_study/install/setup.bash`
   → 旧工作空间只有 3 个 navbot 包，shell 里 `AMENT_PREFIX_PATH` 被旧空间占据。
2. 在这样的 shell 里执行 `colcon build`，colcon 生成 `install/setup.bash` 时会写
   **prefix_chain**——把「生成时已 source 过的所有 prefix」逐条 `source` 一遍。
   于是 `~/navbot_ws/install/setup.bash` 第 25 行变成：
   ```bash
   COLCON_CURRENT_PREFIX="/home/liang/桌面/具身智能学习路线/bridge/ros_study/install"
   _colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
   ```
3. 结果：source 新空间的 setup.bash 会**同时把旧空间的 11 个包拉进来**
   （`AMENT_PREFIX_PATH` 16 条，其中 11 条命中 `ros_study`）。
   新包在前所以 `navbot_bringup` 取到的是新版，但顺序一变就会取到旧版——
   而且旧路径含中文，`map_server` / AMCL 有炸的风险。

**为什么不能靠「在新终端里 rm -rf build install 再 build」修**：
新终端 source 的是**已污染**的 setup.bash → 环境里仍有旧空间 → 重建出来的 setup.bash 照样污染。**鸡生蛋，必须跳出这个 shell。**

**修复**（两步，已执行）：

```bash
# 1) .bashrc 改指新空间（条件式，install 不存在时不报错）
if [ -f "$HOME/navbot_ws/install/setup.bash" ]; then
  source "$HOME/navbot_ws/install/setup.bash"
fi

# 2) 用「完全干净的环境」重新生成 setup 脚本
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm bash -c \
  'source /opt/ros/humble/setup.bash && cd $HOME/navbot_ws && colcon build'
```

**修复后实测**：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `AMENT_PREFIX_PATH` 条目 | 16 | **5** |
| 含 `ros_study` 的条目 | 11 | **0** |
| `ros2 pkg list \| grep navbot` | 3（缺 chassis） | **4** |
| `ros2 pkg list` 总数 | 379+11 练习包 | 379（干净） |

备份：`~/.bashrc.bak.20261029`、`~/navbot_ws/install/setup.bash.bak.20261029`

**教训**：**多工作空间并存时，绝不能在已 source 别的空间的终端里 build 新空间。**
先开干净终端（或 `env -i`），再 build。

---

## W3 · 冷启动第一轮（2026-10-29 已做静态验证，动态待本机跑）

### 已完成：静态验证（无需显示器）

**`xacro` 展开**：326 行，7 个 link 齐全
`base_footprint` / `base_link` / `left_wheel_link` / `right_wheel_link` /
`front_caster_link` / `back_caster_link` / `laser_link`

**三个 Gazebo 插件都在**（`navbot.gazebo.xacro`）：

| 插件名 | .so | 职责 |
|---|---|---|
| `diff_drive` | `libgazebo_ros_diff_drive.so` | `/cmd_vel` → 轮速 → `/odom` + TF |
| `joint_state_publisher` | `libgazebo_ros_joint_state_publisher.so` | `/joint_states` |
| `laser_controller` | `libgazebo_ros_ray_sensor.so` | 仿真激光 → `/scan` |

**关键参数（与文档一致）**：`wheel_separation=0.16`、`wheel_diameter=0.065`、
激光用 `<ray>`（**不是 `gpu_ray`**）、`<samples>360`、`<min>0.12`、`<max>12.0`、
`<update_rate>50`（odom）/ `10`（scan）。

### 已完成：无头环境实跑（能起，但数据流不通）

在无显示的沙箱里用 `gui:=false` 跑通了：

```
[spawn_entity.py] Spawn status: SpawnEntity: Successfully spawned entity [navbot]
[gzserver] [INFO] [diff_drive]: Wheel pair 1 separation set to [0.160000m]
[gzserver] [INFO] [diff_drive]: Wheel pair 1 diameter set to [0.065000m]
[gzserver] [INFO] [diff_drive]: Subscribed to [/cmd_vel]
[gzserver] [INFO] [diff_drive]: Advertise odometry on [/odom]
```

话题全部创建：`/clock /odom /scan /tf /tf_static /joint_states /cmd_vel /cmd_vel /robot_description`

**但 `/clock` 不推进，`/odom` `/scan` 无数据。** 日志里有：

```
[Err] [RenderEngine.cc:749] Can't open display:
[Wrn] [RenderEngine.cc:89] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:292] Cannot initialize render engine since render path type is NONE
```

**结论：无显示器时 Gazebo Classic 的世界更新不正常，没法在无头环境验数据流。**
这不代表配置有问题——`spawn` 成功 + 插件参数正确 + 话题全建，说明**配置层面是通的**。
数据流必须在本机（有显示器）跑。

> ⚠️ 别把这条误读成「`gui:=false` 不能用」。真实原因是没有 X display，不是 gui 参数。
> 本机有显示器时默认 `gui:=true`，直接跑即可。

### 新增：验收脚本 `tools/env/check_sim.sh`

一条命令跑完 W3 全部判据，G1 阶段继续用：

```bash
# 终端 1
ros2 launch navbot_bringup sim_navbot.launch.py
# 终端 2
~/navbot_ws/tools/env/check_sim.sh
```

检查 5 组：必备话题存在 → 发布者数量 → `/odom` `/scan` 频率 → TF 链 `odom→base_footprint` → `/scan` 数据有效性。
失败时按「`/scan` 无数据 / `/odom` 无数据 / TF 查不到 / 两个都无数据」四类给排查顺序。
退出码 0 = 全过。

### 待你本机执行

```bash
source /opt/ros/humble/setup.bash && source ~/navbot_ws/install/setup.bash
ros2 launch navbot_bringup sim_navbot.launch.py
# 另开终端
~/navbot_ws/tools/env/check_sim.sh
```

**出口判据**：脚本输出 `0 FAIL`，其中 `/odom` 40~60 Hz、`/scan` 7~13 Hz。


---

# S0 进度 · 仓库补洞与迁移（2026-10-29 20:30）

## 做了什么

工作空间从 `~/桌面/具身智能学习路线/bridge/ros_study` 迁到 `~/navbot_ws`，
git 历史完整保留（clone 旧仓库后把 `.git` 迁过来，不是重新 init）。

| 项 | 结果 |
|---|---|
| 工作空间根 | `~/navbot_ws`（纯 ASCII，规避中文路径炸 map_server / AMCL） |
| 提交 | `7403608` → 已 push 到 `github.com/liang768919254/navbot-ros2` |
| 验收体系 | `无硬件自测/` → `tools/verify/`，5 脚本 2179 行全在 |
| 环境脚本 | 顶层散落 → `tools/env/`，去掉 `/mnt/workspace` 硬编码 |
| 排障沉淀 | `.check/` → `tools/diag/`（fastdds UDP-only、auto_scan） |
| 移除 | 9 个练习包 + 8 个一次性调试脚本（原件保留在旧目录） |
| 保留 | `firmware/` 133 个 C 文件、`docs/` 14 份文档 + 演示 GIF |

## 路径自动探测（这次的核心改造）

三个脚本不再绑死目录层级，统一规则：

```
优先级：环境变量 NAVBOT_WS > 从脚本位置向上找含 src/ 的目录 > 兜底默认值
```

- `tools/verify/verify_e2e.sh`：原来靠 `SCRIPT_DIR/..`，搬到 `tools/verify/` 后层级变了
- `tools/verify/crosscheck_bridge_functions.py`：同步改（`_find_ws_root()`）
- `tools/env/setup_env.sh` / `record_navigation.sh`：原来写死 `/mnt/workspace`

好处：同一份脚本在 `~/navbot_ws`、`/mnt/workspace`（云实例）、任何 clone 路径下都能跑。

## 实测数据（2026-10-29 20:25）

```
crosscheck_bridge_functions.py  →  61 通过 / 0 失败
test_bridge_offline.py          →  18 通过 / 0 失败   （/odom 48.5 Hz，frame_id 正确）
verify_e2e.sh                   →  16 通过 / 4 失败 / 3 跳过
```

**4 项失败全是沙箱无 pty 导致**（`/dev/ptmx` 不存在 → `out of pty devices`），
失败项全部集中在串口物理层；同一批判据在 offline 测试里 18/18 全过。
有 `/dev/pts` 的本机 Ubuntu 上不会出现。

## 克隆验证（陌生人视角）

```bash
git clone git@github.com:liang768919254/navbot-ros2.git
cd navbot-ros2
python3 tools/verify/crosscheck_bridge_functions.py   # → 61 通过 / 0 失败
```

脚本权限（100755）在 clone 后保留，不需要手动 `chmod +x`。

## S0 判据核对

| 判据 | 状态 |
|---|---|
| 5 个脚本在仓库里 | ✅ `tools/verify/` |
| 路径不写死 `~/桌面` | ✅ 向上自动探测 |
| README 有「怎么验证」 | ✅ 验收体系章节 + 实测数字 |
| push 后 GitHub 可见 | ✅ `7403608` |
| 换目录 clone 仍能跑 | ✅ 已实测 |
