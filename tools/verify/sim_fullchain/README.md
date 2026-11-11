# sim_fullchain —— 仿真全流程验收工具

一条链跑完「仿真 → 建图 → 定位导航 → 动态避障 → RViz 可视化」，每步带可判定的验收判据。

## 为什么单独一套

`tools/env/check_sim.sh` 只验「仿真层通不通」（话题、TF、频率）。
`tools/verify/verify_e2e.sh` 验的是「串口协议链路」（真机侧）。
本目录补的是中间那层：**Gazebo 里的车能不能真的建图、导航、避障**。

## 快速开始

### ★ 推荐：`live.sh` 实时联调（2026-11-10 新增，因为沙箱限制）

一条命令起全套，全程在你自己的终端里，Ctrl+C 一次性全停：

```bash
cd ~/navbot_ws/tools/verify/sim_fullchain

# 建图模式：Gazebo GUI + SLAM + RViz，手动开车看地图实时长
bash live.sh

# 建图 + 自动扫房间（survey 路线约 4 分钟），扫完自动存图
bash live.sh drive

# 导航模式：Gazebo GUI + Nav2 + AMCL + RViz，可以点 Nav2 Goal
bash live.sh nav

# 导航模式 + 自动跑到指定点
bash live.sh nav 2.0 -1.8
```

另开一个终端体检（全实测数值 + PASS/FAIL）：

```bash
bash check_live.sh          # 建图模式
bash check_live.sh nav      # 导航模式
```

### 无人值守批量验收（跑完自动退出）

```bash
cd ~/navbot_ws

# ① 建图（起仿真+SLAM+自动驱车+存图，约 3~5 分钟）
bash tools/verify/sim_fullchain/build_map.sh loop

# ② 定位导航（起仿真+Nav2+注入AMCL初值+导航到目标，约 2 分钟）
bash tools/verify/sim_fullchain/verify_nav.sh

# ③ 动态避障（路径中途插一个地图上没有的箱子）
bash tools/verify/sim_fullchain/verify_avoid.sh

# ④ RViz 可视化截图（产出 shots/01~03）
bash tools/verify/sim_fullchain/capture_rviz.sh
```

> ⚠️ ①~④ 在助手环境里**跑不了完整流程**（见坑 3：跨调用回收 + 120s 超时）。
> 它们的设计前提是「在一个终端里跑完」，这在你自己的终端里没问题。

## 各脚本判据

| 脚本 | 判据 | 上一轮实测 |
|---|---|---|
| `build_map.sh` | 墙像素>200；未知区域<15% | 墙 831，未知 2.5% |
| `verify_nav.sh` | 8 个 Nav2 节点全 active；终点误差<0.25m | 全 active，误差 0.125m |
| `verify_avoid.sh` | 激光看到障碍+代价图标记+绕开>0.30m+仍到目标 | 0.166/911格/0.371m/0.121m（S36 全 PASS） |

> ★ **避障用例的场地要用「可通行带」算，不能凭感觉选**：
> `可通行带 = (车心净空 − inflation) − (障碍半宽 + inflation)`
> 以 `inflation=0.16`、障碍半宽 `0.15` 计：
>
> | 场地 | 净空 | 可通行带 | 结果 |
> |---|---|---|---|
> | 南走廊 | 0.450 | −0.020 | 数学上不可行 |
> | 北走廊 | 0.550 | +0.080 | 实测卡死 60s、`failed to create plan`×2 |
> | 东侧 x=2.30（现用） | 0.650 | +0.180 | 一次 PASS |
>
> `avoid_test.py` 已内置闸门：带宽 < 0.10 m 直接作废（返回码 6）。
> 换目标点前先用 `room_geom.clearance(x, y)` 算一下，别跑完 4 分钟才发现。

## ★ 六个踩过的坑（都已在脚本里规避）

### 0. `bash drive.py` 是错的——那是把 Python 当 shell 脚本（2026-11-10 小梁同学踩到）
```
$ bash drive.py loop
drive.py: 行 18: from: 未找到命令
drive.py: 行 24: `class Driver(Node):' 附近有语法错误
```
`bash` 逐行当 shell 读 `.py`，把 `from`/`class` 当命令找。
顺带还会冒出 `import-im6.q16: ... security policy PS` ——
那是 ImageMagick 的 `import` 命令被执行了（`from ... import ...` 里的 `import` 被当命令）。

正确写法：
```bash
./drive.py loop            # 靠 shebang + 可执行权限
python3 drive.py loop      # 等价
```
**并且**要先 source ROS 环境，否则 `import rclpy` 直接失败。

`drive.py` 现在自带三层自检，三种误用都会给可执行指引而非 traceback：

| 误用 | 输出 |
|---|---|
| 没 source ROS | 提示 `source .../env.sh` 的完整命令，退出码 2 |
| 路线名写错（如 `square`） | 列出 `loop \| survey \| home`，退出码 2 |
| `/odom` 20s 无数据 | 三步排查顺序，**不开车**（避免无闭环盲撞） |
| `timeout`/`kill` 打断 | 一行「收到外部终止信号」，不再是一屏 traceback |

### 0.1 `check_live.sh` 不能用 `pgrep` 判进程（同 PID namespace 隔离）
在助手沙箱里跑体检时，`pgrep -x gzserver` 看不到小梁终端起的进程
（沙箱有独立 PID namespace），导致「4 项进程全 FAIL，但话题/TF/地图全 PASS」
这种自相矛盾的输出。→ 改查 **ROS 图**（`ros2 node list` 里有没有
`/gazebo`、`/diff_drive`、`/slam_toolbox`），跨 namespace 有效。

### 1. `python3` 被解析成 3.13，ROS 节点全废
本机 PATH 前面有托管的 python 3.13，而 ROS Humble 的 Python 节点绑死系统 3.10。
症状：`spawn_entity.py` 报 `ModuleNotFoundError: No module named 'numpy'`，
`ros2 topic list` 空、`ros2 topic hz` 无输出。
→ `env.sh` 把 `/usr/bin` 提到 PATH 最前，并自检 `python3 == 3.10`。

### 2. `/dev/shm` 只有 10MB，FastDDS 跨进程发现失败
症状：gzserver 起了、插件日志说已 publish，但另一个终端 `ros2 topic list` 全空。
→ `env.sh` 设 `FASTRTPS_DEFAULT_PROFILES_FILE=tools/diag/fastdds_udp_only.xml` 强制走 UDP。

### 3. Bash 工具每次调用结束回收子进程（★ 2026-11-10 复查后确认仍成立）
本机 Bash 沙箱在每次工具调用返回时把整个进程组回收，
`nohup`+`setsid`+`disown`+`systemd-run --user` 全都留不住进程。

**2026-11-10 探针实证**（推翻了 11-09 那次「能跨调用存活」的判断）：
```bash
setsid nohup bash -c 'echo START > probe.txt; sleep 60; echo END >> probe.txt' &
# 同一调用内: probe.txt 有 START，进程组 4 存活，sleep 60 在跑
# 下一次调用: 进程已消失，probe.txt 里永远只有 START
```
`systemd-run --user` 也不可用：沙箱里没有 D-Bus
（`Failed to connect to bus: No such file or directory`）。

连带后果：**一条调用内跑完整链路会撞工具默认 120s 超时被 SIGTERM**。
实测：仿真 14s + SLAM 12s + 建图 240s = 266s，跑到 240s 建图中途被杀，
存下来的地图未知区域 18.4%（完整图是 2.5%）。

→ 结论：**助手只能做「一次性短任务」**（单次 <100s，如起仿真+读数、验参数）。
  凡是需要长时间连续跑 + 多轮观察的，一律交给你在自己的终端跑（见 `live.sh`）。


### 4. `/initialpose` 不能用 `ros2 topic pub --once` 发
`--once` 发完进程立刻退出，DDS 投递未完成，AMCL 收不到。
症状：`amcl` 一直刷 `AMCL cannot publish a pose... Please set the initial pose`。
→ `set_pose.py` 持续发 3 秒（15 次），发完还 spin 1.5 秒等 AMCL 处理。

### 5. 必须先给 AMCL 初值，Nav2 才会激活
`lifecycle_manager` 按序激活，`planner_server` 的 `global_costmap` 在 activate
阶段要 `Checking transform (map→odom)`。此时 AMCL 若没初值 → `map→odom` 不存在
→ 卡死 → 后面的 `bt_navigator` 永远 inactive。
症状：`bt_navigator` 一直 `inactive [2]`，`planner_server` 查不到，
但 `controller_server` 是 active，导航"看起来能跑"（其实是残缺链路）。
→ 脚本顺序固定为：起 Nav2 → 注入初值 → 等 active。

### 6. `SpawnEntity` 的 `initial_pose` 必须显式给坐标
`sdf` 里的 `<pose>` 和服务的 `initial_pose` 会**叠加**。
只写 `<pose>1 0 0.25 ...</pose>` 而 `initial_pose` 留默认全零，
箱子会落在世界原点 `(0,0)`——正好和车初始位置重叠，表现为
「spawn 返回 success=True，但激光完全看不到、Nav2 也不标记」。
→ 统一做法：SDF 里 `<pose>0 0 0 0 0 0</pose>`，位置只由
`initial_pose` + `reference_frame: 'world'` 决定。

## 环境相关的非致命现象

这些**不是 bug**，出现属正常：

| 现象 | 原因 |
|---|---|
| `gzclient` 崩溃 `libGL error: failed to load driver: iris` | **这个判断 2026-11-09 被推翻**：本机 `/dev/dri/card1` + `renderD128` 都在，`glxinfo` 报 `Accelerated: yes / Mesa Intel(R) Graphics (RPL-P) / OpenGL 4.6`。真正原因是 `env.sh` 没 source `/usr/share/gazebo/setup.sh` → `GAZEBO_RESOURCE_PATH` 为空 → OGRE 找不到 shader 库。**已在 env.sh 修掉** |
| 助手环境里 gzserver 段错误 `Unable to create the rendering window` | `ulimit -v` = 4194304 KB（4GB，soft=hard），Gazebo 的 OGRE+Mesa 要预留几十 GB 地址空间。**你的终端是 unlimited，不受影响** |
| RViz 退出时报「段错误（核心已转储）」 | 日志最后是 `signal_handler(SIGINT/SIGTERM)`，段错误发生在**退出清理阶段**（OGRE 释放资源）。运行期与截图均正常 |
| RViz 报 `indexed_8bit_image ... active samplers with a different type` | llvmpipe 下地图图层的 GLSL 着色器兼容警告，**渲染结果正确** |
| `ALSA / OpenAL: Unable to open audio device` | 无声卡，Gazebo 自动禁用音频 |
| `Failed to create /root/.cache for shader cache` | 沙箱下 HOME=/root 不可写，着色器缓存禁用，不影响 |

## 文件说明

| 文件 | 作用 |
|---|---|
| `live.sh` | ★ **实时联调总入口**：Gazebo GUI + SLAM/Nav2 + RViz 一条命令起全套（在你的终端跑） |
| `check_live.sh` | ★ **体检**：进程/激光/TF/地图/Scale/Nav2/代价地图/AMCL 逐项 PASS-FAIL |
| `env.sh` | 统一环境入口：剔托管 python、系统路径置顶、ROS source、UDP DDS、渲染后端可切 |
| `run_gui.sh` | 只起 Gazebo GUI（不带 SLAM/RViz） |
| `run_rviz.sh` | 只起 RViz（`nav` / `slam` / 自定义配置三种） |
| `run_bg.sh` / `stop_bg.sh` | 后台起/停进程（**同一脚本内**有效；`stack` 模式保留 Gazebo 与你自己起的 RViz） |
| `drive.py` | 按 `/odom` 闭环驱车（`loop` 绕圈 / `survey` 扫房间 / `home` 回原点） |
| `odom_pose.py` | ⚠️ **已弃用**：输出 `/odom` 位姿，仅供手动排查。**别再拿它当 AMCL 初值**（map/odom 会漂，S34 实测差 1.4 m） |
| `set_pose.py` | ★ **注入 AMCL 初值**：`--map x y yaw` 显式指定 / `--auto` 按 `map->odom` 反推（推荐）。注入后反查 `map->odom` 验证，不健康自动重注 —— 「消息发出去了」≠「AMCL 收敛了」 |
| `loc_check.py` | ★ **定位健康检查**：用 rclpy 直查 `map->odom`，判 z / roll / pitch 是否在平面内、AMCL 是否冻结、odom-map 漂移。**不要用 shell 解析 `tf2_echo` 文本**（每 1 s 才打一帧，`head -20` 会抓到多帧拼成多行，判定静默失效） |
| `room_geom.py` | ★ **几何真值唯一来源**：房间/隔板/立柱尺寸、通道净空、膨胀代价带。`--selftest` 验恒等式 |
| `nav_patrol.py` | ★ **8 点巡检**：`--dry` 离线预检 / `--plan` A* 预演 / `--selftest` / 实跑 |
| `nav_precheck.py` | ★ **导航前体检**：地图/TF 链/6 个 lifecycle 节点/AMCL 收敛/当前格代价，全绿再发目标 |
| `map_quality.py` | ★ **地图质检**：真值 vs 实测逐条比对。`--offline <pgm>` 不启 ROS 也能算 |
| `build_map.sh` | 建图全流程 + 地图质量判据 |
| `verify_nav.sh` | 导航全流程 + 生命周期与终点误差判据 |
| `avoid_test.py` / `verify_avoid.sh` | 动态避障（路径中途插入未建图障碍） |
| `capture_rviz.sh` | 分阶段 RViz 截图 |

## 导航模式标准操作流程（2026-11-10 起）

之所以从「两步」变成「四步」：`live.sh nav` 里 `sleep 12` 是个赌注，
机器忙时 AMCL 的 lifecycle 还没 active，`/initialpose` 发出去没人接，
表现为 AMCL 粒子云撒在地图原点附近、planning 一路 abort，
而日志只写「no valid path」——看起来像规划器坏了，实际是初值丢了。
所以别赌时间，直接问节点。

```bash
# 终端 1：起 Gazebo + Nav2 + AMCL + RViz（保持不动，Ctrl+C 全停）
cd ~/navbot_ws/tools/verify/sim_fullchain
bash live.sh nav

# 终端 2：先体检，全绿再说
source ~/navbot_ws/tools/verify/sim_fullchain/env.sh
python3 nav_precheck.py --wait 90

# 终端 2：体检过了才跑 8 点巡检
python3 nav_patrol.py
```

体检 5 组判据：

| # | 检查 | FAIL 说明什么 |
|---|------|--------------|
| 1 | `/map` 收到、分辨率 0.05、覆盖全房间 | map_server 没起 / 地图 yaml 路径错 |
| 2 | 6 个 lifecycle 节点都 active | autostart 失效（`use_composition` 必须是 False） |
| 3 | TF `map→odom→base_footprint` 连成链 | slam_toolbox 还在跑、抢了 `map→odom` |
| 4 | AMCL 与 odom 偏差 < 0.30 m、位置方差 < 0.10 | **初值没生效**，粒子还散着 |
| 5 | 车当前格不是 lethal、净空 > 车半径 | 车停在了非法位置（上次导航残留） |

不开仿真也能先验的部分（全部离线）：

```bash
python3 room_geom.py --selftest      # 几何真值恒等式 + 四层门槛 + 车心可达域
python3 nav_patrol.py --selftest     # 路线/连通性/通宽/绕行必要性
python3 nav_patrol.py --dry          # 8 点逐点查地图格 + 净空
python3 nav_patrol.py --plan         # A* 预演：段长/最小净空/链式漂移/可达域
python3 nav_precheck.py --selftest   # 体检脚本自身逻辑
python3 map_quality.py --offline ~/navbot_ws/src/navbot_bringup/maps/navbot_room.pgm
python3 ../../verify/check_limits.py # 两处 costmap 参数一致性 + 0.6m 通道红线
```

## ★ 车「顶着墙不动」怎么查（2026-11-10 立）

巡检出现 `ABORTED` 或看起来「撞墙卡住」时，**先分清是控制器还是规划器**，
别看现象猜。一行命令定性：

```bash
grep -c "Passing new path to controller" live_nav.log
grep -c "Reached the goal"              live_nav.log
```

- **前者 >> 后者**：路径每秒都在刷新、规划器没失败 ⇒ **控制器不敢执行**（第③④层）
- **前者 == 0**：根本没路径 ⇒ 查 `planner_server` 报错（第①②层）

本项目实测是前者（`Passing new path` 一直打，`Failed to make progress` 循环 17 次后 ABORTED），
说明 DWB 不是撞墙，而是**算不出比 v=0 更好的解**。

四层门槛（从硬到软，要逐层过）：

| 层 | 判据 | 本项目 |
|---|---|---|
| ① 物理能过 | `缝宽 > 2 × robot_radius` (0.400) | 缝E 0.300 → 死路 |
| ② 能规划 | 同上，非致命带 > 0 | 缝N 0.450 ✓ |
| ③ 愿走 | `缝宽 > 2 × inflation_radius` | 旧 0.50 → 缝N 封死；新 0.44 → 露出 10 mm |
| ④ 敢停 | 目标到达球内有合法车心位置 | 东北角——`xy_goal_tolerance` 0.15→0.12 |

> 完整推导、日志证据、以及我排查时犯的三个错，见 `docs/S27_导航卡住的四层病因.md`。

`nav_patrol.py --plan` 现在会打印每段的「可达域」列：
如果某个点的可达域接近 `TOL_ERR`，那个点就算 SUCCEEDED 也可能超判据，
在预检阶段就能看出来，不用等跑完 8 点。

## ★ 导航版 RViz 为什么曾比建图版糊（2026-11-10 修）

小梁同学的原话：「闭环跑的 rviz 比导航起的 rviz 清晰多了，而且导航起的 rviz
它的障碍物特别粗」。查下来**激光参数两边完全一样**（Points / Size 3 / Decay 0），
差的是另外两层：

| 差异 | 建图版 `navbot_slam.rviz` | 导航版改前 | 视觉后果 |
|------|--------------------------|-----------|---------|
| `/map` Alpha | 0.7 | 0.7 | 半透明 → 黑墙变深灰，叠在 48,48,48 背景上对比度掉一档 |
| `GlobalCostmap` | 无 | **默认开启** Alpha 0.5，画在地图**之上** | 墙两侧各糊 0.45 m 蓝紫带（inflation 0.25 + cost_scaling 5.0），盖住真墙 |
| `LocalCostmap` | 无 | **默认开启** Alpha 0.5 | 车周围额外糊一个 3×3 m 方块 |
| 叠加层数 | 1 层 | 3 层半透明 | 整屏发灰发糊 |
| 激光 Size | 3 px 绿 | 3 px 红 | 红色压在蓝紫底上，视觉上被吃掉 |

那 0.45 m 是怎么来的（不是随手填的）：

```
inflation_radius = 0.25        → 代价带外沿到障碍 0.25 m
cost_scaling_factor = 5.0      → cost = 253·exp(-5(d - r))
cost 衰减到 RViz 仍能看出颜色 ≈ 还有 0.2 m
                              ⇒ 合计约 0.45 m
```

### 改法（没有直接换用建图版配置）

直接换是不行的——建图版没有 `nav2_rviz_plugins/Navigation 2` 面板、
没有 `SetInitialPose` / `GoalTool`，换了就没法点目标了。
所以保留导航版全部功能，只修显示参数：

| 项 | 改后 | 理由 |
|---|------|------|
| `/map` Alpha | 0.7 → **1.0** | 代价图不再压在上面了，地图该是实心的 |
| `GlobalCostmap` / `LocalCostmap` | 默认 **关**，Alpha 0.35，`Draw Behind: true` | 想看时勾一下；勾上也不挡墙 |
| 激光 `Size (Pixels)` | 3 → **5** | 补偿红色对比度；上限 7，否则开始糊（Scale 150 时 1 px ≈ 6.7 mm） |

保留不动：`GlobalPlan` 绿线、`LocalPlan` 橙线、`Navigation 2` 面板、
`SetInitialPose`、`GoalTool`、`Fixed Frame: map`、`Scale: 150`。
`ParticleCloud` 仍默认关——2000 个 3D 箭头在核显上会拖死 GUI。

### ⚠️ 改 `.rviz` 后必须生效的两件事

1. **重启 RViz**（配置只在启动时读一次，运行中改文件无效）
2. **install 必须同步**。原来 `install/` 下是实体拷贝，改 `src/` 不 build 就白改。
   2026-11-10 已改用 `--symlink-install`，现在 `install/` 下是软链接：

   ```bash
   ls -l ~/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/
   # navbot_nav.rviz -> /home/liang/navbot_ws/src/navbot_bringup/rviz/navbot_nav.rviz
   ```

   以后改 `src/` 下的 rviz 配置**不用再 build**。若哪天又变回实体文件，
   说明那次 build 没带 `--symlink-install`，补一次即可。

原配置已备份在 `src/navbot_bringup/rviz/.bak/navbot_nav_20261110_1436.rviz`。

## ★ 关于 RViz Scale（2026-11-09 修的真 bug）

`navbot_nav.rviz` / `navbot_slam.rviz` 里 `TopDownOrtho` 的 `Scale: 150`（像素/米）。

- 150 → 窗口 1500px 宽 → 视野 10m，6m×5m 房间铺满大半窗口 ✅
- 60 → 视野 25m，房间只有 360px 宽，看着像缩在角落 ❌

**这个改动之前一直没生效**，原因是 `install/` 下是**拷贝**不是 symlink：
改 `src/navbot_bringup/rviz/*.rviz` 后不 `colcon build` 或 `cp`，
RViz 读的还是 `install/` 里的旧文件。

→ 改完必查：
```bash
grep -A1 "TopDownOrtho" install/navbot_bringup/share/navbot_bringup/rviz/navbot_nav.rviz
# 下一行必须是 Scale: 150
```
