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

---

# H0 · 硬件勘察脚本（2026-10-31）

重启真机阶段先解决「这台车到底有什么」——没有可信的硬件快照，后面每个「没数据」都得靠猜。

## 交付

`tools/env/check_hw.py`：只读、零第三方依赖、Python 3.8+。

| 采集项 | 内容 |
|---|---|
| 板卡 | device-tree model（树莓派/SBC）、DMI（x86）、架构 |
| 计算 | CPU 型号 / 物理核 / 线程 / 当前频率 / 负载 / 内存 / 交换 |
| 传感 | thermal_zone 温度（含 WiFi 芯片）、树莓派 `vcgencmd` 欠压标志 |
| 供电 | 电池电量与状态、AC、`get_throttled` |
| 存储 | lsblk 树（滤掉 loop），容量 / 介质 / 挂载点 |
| 外设 | USB 全量（VID:PID + 已知设备表）、串口（芯片 / 权限 / by-id）、I2C / SPI / 摄像头 / GPIO |
| 网络 | 每网卡类型 / MAC / IP / 驱动 / 速率、默认路由、真实 DNS（绕开 resolved stub） |
| WiFi | SSID / BSSID / 信道 / 频段 / 信号 dBm / 速率 / 加密 / AP 能力 / rfkill / 监管域 |
| ROS 2 | distro / RMW / domain、prefix 数、5 个关键包是否可见 |

## 三个设计取舍

1. **信号值三路降级**：`iw` → `/proc/net/wireless`（不依赖任何工具，最稳）→ nmcli 估算。
   本机实测无 `iw`、nmcli 连不上 NM，仍拿到 -47 dBm。
2. **每个采集器异常隔离**：`collect()` 用 `safe()` 包裹，单模块失败只记一条告警，
   报告照常出完。真机 sysfs 千奇百怪，不能因为一个模块崩掉整份快照。
3. **给判据而不只是罗列**：末尾 7 项「上路就绪」，`--strict` 可当门禁用。

## 雷达判据的坑

初版把任何 CH340 都算作雷达，导致「只插底盘」也报雷达 OK。
改成 CP210x/FT232 优先；CH340 只在出现 **2 个及以上** 时把第二个算作雷达候选
（山寨转接板确实用 CH340）。四种组合已逐个验过。

## 实测（2026-10-31 22:27，本机无硬件）

```
check_hw.py       → 达标 3/7（缺底盘串口、雷达；ROS 未 source）
--quiet           → 只列 4 项未达标
--json -          → 15 个顶层键，JSON 合法
--link-test       → 网关 RTT 3.75 ms / 公网 21.47 ms，丢包 0%
--scan            → nmcli 不可用时优雅报错，不崩
空环境（全 mock）  → 0 处裸 None
耗时 0.14 s
```

## ESP8266 AT 探测（2026-10-31 追加）

小梁确认：底盘 PCB 上焊死的 WiFi 模块是 **ESP8266**。核代码后确认接线现状：

| 事实 | 证据 |
|---|---|
| USART1(PA9/PA10) → USB-TTL | `usart.h:39-49`，`USART_UX` = `USART1` |
| USART3(PB10/PB11) → RPLIDAR A1 | `rplidar.h:34` |
| 全工程无 USART2 初始化、无 ESP/AT 代码 | grep 业务代码 0 处 |

F103C8T6 三路 USART，前两路占满 → ESP 只能挂 USART2（固件未透传）或直连板载 USB-TTL。

### 新增 `--wifi-module [PORT]`

只发查询类 AT，**绝不动状态**：

```
AT → AT+GMR → AT+CWMODE? → AT+CWJAP? → AT+CIPSTA? → AT+CIFSR
```

不发 `AT+RST`（重启）、`AT+CWJAP=`（连 AP）、`AT+CWMODE=`（改模式）、`AT+CIPSEND`（发数据）。
`--scan` 时补一条 `AT+CWLAP`。

失败分三种给不同诊断，不让用户猜：

| 现象 | 诊断 |
|---|---|
| 端口被 fuser/lsof 占着 | 先停 navbot_bridge |
| 串口能开但 AT 无应答 | 可能挂在 STM32 USART2，需固件透传 |
| 串口打不开 | 设备不存在或不在 dialout 组 |

### 测试 `tools/verify/test_esp_at_probe.py`

分两层，互不依赖：

| 层 | 内容 | 环境要求 |
|---|---|---|
| A 解析层 | 15 项，喂真实 AT 响应文本，验字段提取 + 3 个边界（未连 AP / 空响应 / 截断报文） | 任何环境，**沙箱实测 15/15** |
| B 收发层 | pty 造一个必定应答的假 ESP8266，跑完整链路 | 需 `/dev/ptmx`，沙箱无 pty 时 SKIP(77) |

解析逻辑专门抽成 `_parse_esp_responses()` 就是为了能脱机单测——**AT 解析写错的表现和「模块没接」一模一样**，
不脱机测一次根本分不清。

## 真机首跑暴露的 bug（2026-10-31 22:41）

小梁在本机跑测试，A 层 15/15 过、B 层（当时只有 pty）**14 项全 FAIL，耗时 0.07s**。

`0.07s` 是决定性的：三个波特率 × 五条指令，光第一条 `AT` 的等待窗口就有 0.6s，
根本来不及发。说明它**一条指令都没发就退出了**。

根因在 `_port_busy()`，两处错叠在一起：

| bug | 原因 |
|---|---|
| 把路径里的数字当成 PID | `re.findall(r"\d+", out)` 会抓路径里的数字（`/dev/pts/1` 里的 `1`） |
| 自己把自己判成占用 | 没排除 `os.getpid()`。pty 测试里探测进程自己握着 master/slave 两个 fd，`fuser` 一查就有它 |

命中 busy 后代码 `continue` 掉整个端口 → `found=False` → 看起来和「硬件没接」完全一样。

**第一次修**是「按冒号切开只取 PID 段」，自以为对了。实测 fuser 才发现格式根本不是那样：

```
$ fuser /tmp/busy_probe.txt
     4/tmp/busy_probe.txt:          ← PID 在路径**前面**，和常见的 `/path: 1234` 正好相反
```

按冒号切只能拿到空串，于是「碰巧」返回 None——**真被别的进程占用时照样检测不出来**，
等于修了个假的。

**最终改法**：不解析任何外部命令的输出，直接扫 `/proc/*/fd` 比 `realpath`。
好处是格式完全可控、软链（`/dev/serial/by-id`）也能匹配、天然跳过自己；
读不到别人 fd 目录时静默跳过（权限不足宁可漏报，不要误报）。

四个场景已固化进测试：自己持有→空闲、别人持有→点出 PID、释放后→空闲、不存在路径→安全返回。

**同时补了 B 注入层**：用 socketpair 冒充串口 fd（monkeypatch `_open_tty`），
让「端口检测 → 试探 → 查询 → 解析」这条主流程在无 pty 的环境也能验。
原设计只有 pty 一层，环境一缺就整层 SKIP，等于没测——**这正是这个 bug 能活到真机的原因**。

沙箱实测：A 15/15、A2 4/4、B 16/16、C 因无 pty 跳过。

### 顺带确认的接线

真机 `lsusb` 只有 **1 颗 CH340**（`1a86:7523`），据此判定 ESP8266 走的是
「挂在 STM32 后面」而不是「直连 USB-TTL」，即 `docs/H1` 里的情况 B。
在 STM32 固件把 AT 透传做出来之前，`--wifi-module` 对他这块板不会有结果——这不是脚本的问题。

### 顺带修的

`_wrap()` 原来按字符硬折，把 `STM32` 切成 `STM` + `32`。改成优先断在空格，
且空格位置小于 1/3 行宽时就不断（避免出现极短行）。

### 接线确认与固件方案

见 [`docs/H1_ESP8266接线确认与AT透传方案.md`](docs/H1_ESP8266接线确认与AT透传方案.md)：
数转串口芯片颗数 → 确认 PA2/PA3 交叉接 → 需要透传时给骨架（**只给结构 + 6 条验收判据，不代写实现**）。

## 新增：串口身份识别（静听）

`--wifi-module` 原来只会说「AT 无应答」，这句话对排查没用 —— 用户没法判断是插错口、
模块没供电，还是没做透传。

改成先**静听 0.8 秒、一个字节都不发**，再发 `AT`。这招能成立是因为两者行为完全不同：

| 对面 | 静听结果 |
|---|---|
| navbot 的 STM32 底盘 | 每 50 Hz 主动吐 O 帧，静听必有数据 |
| ESP8266（AT 固件） | 纯被动，不问不说话，静听为空 |

`_classify_port_reply()` 分六类：`esp` / `navbot_stm32` / `stm32_debug` / `garbled` / `text` / `None`。
认出是底盘就**提前 break**，不再试其它波特率、也不再发 AT。

输出从「AT 无应答」变成：

```
【底盘 WiFi 模块（ESP8266 / AT）】
  对面是  : STM32 底盘（正在发 O/V 帧）  —  /dev/ttyUSB0 @ 115200
  结果    : 没探到 ESP8266（试过 1 组端口×波特率组合）
  诊断    : /dev/ttyUSB0 上是 STM32 底盘（收到了 O/V 帧遥测），不是 ESP8266。
            ESP 挂在 STM32 的 USART2 后面，要读它得先由固件做 AT 透传。
```

> 看到 `STM32 底盘` 其实是**好消息**：说明 USART1 链路通、底盘固件在跑，只是 ESP 不在这条线上。

测试新增 A3 层 9/9（7 种文本分类 + 端到端：socketpair 持续发 O 帧，验证认出底盘且不误报 `found`）。

## 真机第二次跑暴露的四个问题（2026-10-31 22:52）

小梁插上板子跑 `--wifi-module`，输出里同时暴露四件事：

| # | 现象 | 根因 |
|---|---|---|
| 1 | `@9600` 收到满屏乱码，却标成「可读文本」 | `decode("utf-8","replace")` 产生的 U+FFFD（`�`）**`isprintable()` 返回 True**，按 isprintable 数比例永远判不出乱码 |
| 2 | `@74880` 报「打不开（权限或设备不存在）」 | 74880 是非标准波特率，`termios` 没有 `B74880`，但报错话术把方向带偏了 |
| 3 | `/dev/ttyS0` 三个波特率全「打不开」，而它明明列在「可读写」 | 真实原因是 `ENODEV(19)`——主板没有实际 UART 硬件。错误被吞成一句笼统话术 |
| 4 | `@115200` 静默 | **固件不是持续输出的**，见下 |

### 4 是真问题，前 3 个是脚本的锅

翻固件源码（`User/main.c:64`）：

```c
usart_init(115200);   /* 调试串口波特率, 复位一上电就能看到日志 */
```

波特率就是 115200，没错。但 R4 固件只在**复位时**打一次 banner，之后沉默。
这种设备盲试 AT 永远试不出来，静听也会错过——不是接错线。

### 修法

1. **乱码判据重写**：主判据改成「U+FFFD 占比 > 8%」，辅以「控制字符 > 20%」。
   不再用 ASCII 比例——那会把纯中文输出误判成乱码。
2. **74880 直接移除**，并在注释里写明原因（要它得用 termios2 + BOTHER，为 boot log 不值得）。
   候选改成 `115200 / 9600 / 57600 / 38400 / 230400 / 460800`，全是标准常量。
3. **新增 `_open_fail_reason()`**：`_open_tty` 失败后反查真实原因——节点不存在 / 无权限 /
   非标准波特率 / `open 失败：No such device（errno 19）`。`ttyS0` 的 ENODEV 现在能一眼看出。
4. **新增 `--baudscan [PORT]`**：逐个波特率静听 2 秒，报每档收到多少字节、像什么。
   配合「扫描时按一下复位键」就能反推真实波特率。

### 新增判读技巧

`@9600` 收到 `6b 6b 6b 6b`（连续四个 `k`）= **过采样特征**，说明线路上确实有数据，
只是波特率没对上。反过来全 0 字节才是真没数据。这条已经写进诊断话术里。

### 顺带

- I2C 节点 15 个刷屏 → `_fmt_list()` 改成「15 个（/dev/i2c-0、…）」
- 摄像头名被 v4l2-ctl 的重复字段撑爆 → 只取第一个冒号前
- 静听窗口 0.8 → 1.0 秒（`ESP_LISTEN_SEC` 常量）

## `--baudscan` 首次实测：数据是间歇的（2026-10-31 23:00）

小梁跑 `--baudscan`（六档各 2 秒），结果很说明问题：

```
115200 : 0 字节          9600   : 0 字节
57600  : 1210 字节 ← 乱码  38400  : 0 字节
230400 : 0 字节          460800 : 0 字节
```

**只有 57600 撞到数据，而且是错的波特率。** 两个结论：

1. 线路上确实在发数据（605 B/s，不是噪声、不是没接线）
2. **数据是间歇发的**——2 秒窗口太小，谁撞上谁有，跟波特率对不对关系不大

hex 也佐证：`28 0c 6d 93 39 2d a1 88 ...` 一半字节高位为 1、一半是可打印字符，
正是「115200 的数据被 57600 采样」的样子（差 2 倍，丢一半 bit）。

### 新增 `--listen`

2 秒窗口的思路本身有问题。加了个长静听模式：

```bash
python3 check_hw.py --listen --baud 115200 --sec 30
```

指定波特率、开长窗口、**期间按复位键让固件重新说话**，然后把收到的原始数据
按文本分行 dump 出来 + 前 96 字节的 hex。

三个参数各有默认值（`auto` / 115200 / 15 秒），不加参数也能直接跑。

### `--baudscan` 的结论话术也改了

原来只会说「没有一档收到可读内容」。现在会分辨「全 0 字节」和「有数据量但读不懂」：

- 后者直接提示：「数据量说明对面确实在发，只是这档采样错了。
  若已知固件波特率，直接用它长听：`--listen --baud <值> --sec 30`」

## `--listen` 首跑：抓到了 R5 固件（2026-10-31 23:04）

30 秒静听收到 6068 字节（202 B/s），内容完全可读：

```
R5 SCAN  --  RPLIDAR A1 (USART3) + SG90 (TIM1_CH1/PA8)
USART3 regs: CR1=0x202C BRR=0x0138  (UE=1 RE=1 RXNEIE=1)
NVIC: USART3_IRQn=39 enabled=1   GPIOB_CRH=0x44448B77 (PB11 is bit12-15)
servo not initialized (SCAN_SERVO_ENABLE = 0)
start scan...
```

**结论 1：115200 完全正确。** 不用再猜波特率了。

**结论 2：`BRR=0x0138` 反算是 36 MHz / 312 = 115384 ≈ 115200**，和 RPLIDAR A1 默认值一致。

### 判据错了一处，已修

「雷达链路」原来按「有没有 CP210x/FT232」判断——**对这块板是错的**。
雷达直接接 STM32 的 USART3（PB10/PB11），根本不经过 USB，Linux 侧永远看不到。

改成三态：

| 情况 | 判定 |
|---|---|
| 找到 CP210x/FT232 | ✓ 走 USB 转接板 |
| 只有 MCU 调试口 | **? 待确认**（可能接 MCU 的 UART，Linux 侧不可见） |
| 什么都没有 | ✗ |

`--strict` 只对明确 ✗ 返回 1，「待确认」不算失败——否则门禁会被不可见项卡死。

### 新增 `--save`

终端只显示前 40 行，而固件一次打印上百行。`--listen --save FILE` 把**完整原始字节**
另存一份，配合 `--sec 30` 就能拿到完整报告。

### 雷达为什么没数据：已定位到雷达侧

见 [`docs/H2_雷达链路联调结论.md`](docs/H2_雷达链路联调结论.md)。
用之前存的 `02_联调工具/raw_ttyUSB0.bin` 交叉验证：

```
CHANNEL rx=0 B  overflow=0  points=0  resync=0
HEAD: <none>
```

`rx=0` + `resync=0` 同时成立 = 线路上没有任何电平变化（不是波特率错，那样 resync 会累加）。
三层排查：PC↔STM32 ✓ / USART3 配置 ✓ / **STM32←雷达 ✗**。下一步量接线。

### 固件自检的两处判据缺陷（读 `radar_scan.c` 确认）

三轮扫描都是 `1/6 PASS`，唯一 PASS 的 T6 还是假阳性：

```
[FAIL] T1 pose samples  0 0 0 0 0 (need >= 2000 each)
[PASS] T6 no rx overflow  0 (need 0)
```

**缺陷一**：`radar_scan.c:343-349` 里 T1 失败固定提示「舵机没到位 / SG90 没接 PA8 /
这一档丢字节」。三条全假设「有数据但点少」，可实际 `rx=0`——一个字节都没有；
而且 `SCAN_SERVO_ENABLE=0`，舵机压根没初始化，前两条不可能成立，
第三条又被 T6 的 PASS 排除。**三条提示全部走不通，只会把人往舵机上引。**

**缺陷二**：`overflow==0` 在 `rx==0` 时必然成立，T6 的 PASS 是假阳性，
制造出「还有一项是好的」的错觉。`rx==0` 时应标 `N/A` 不计入通过项。

判据顺序应为：`rx==0`（有无数据）→ `HEAD`（帧头对不对）→ points 少（才轮到舵机/遮挡）。
「没数据」和「数据不够」是两类故障，不能共用一套提示。

---

# W2' 补 · 仿真全链路跑通（2026-11-09）

把「Gazebo 里能不能真的建图、导航、避障」这一层补上并逐关验收。
工具落在 `tools/verify/sim_fullchain/`，用法与踩坑见其 `README.md`。

## 五关验收结果

| 关 | 内容 | 判据 | 实测 | 结果 |
|---|---|---|---|---|
| 1 | 四包编译 + xacro | 4 包通过、7 个 link | 4 包 1.53s、326 行、7 link | ✅ |
| 2 | 底盘运动学 | `/scan`≈10Hz、`/odom`≈50Hz、TF 闭合 | 9.98Hz / 49.95Hz / 闭合 | ✅ |
| 3 | SLAM 建图 | 墙>200、未知<15% | 墙 831、未知 2.5% | ✅ |
| 4 | Nav2 定位导航 | 8 节点 active、终点误差<0.25m | 全 active、0.142m | ✅ |
| 5 | 动态避障 | 激光看见+代价图标记+绕开+仍到目标 | 0.308m/205格/0.490m/0.207m | ✅ |
| 6 | RViz2 可视化 | 地图/代价图/路径/激光可见 | 三张截图见 `docs/sim_shots/` | ✅ |

## 地图重建

原 `navbot_room.pgm` 是 119×99，新建成 **120×100**，未知区域从 4001 格降到 **294 格（2.5%）**。
布局与 `navbot_room.world` 逐项对上：四面外墙 / `partition_mid` / `partition_short` /
`pillar_a` / `pillar_b` / `movable_box`。

建图路线（`drive.py loop`）修正过两次：
- v1 从原点直接向东 → 撞 `partition_short`（它的上端 y=0 刚好压住起点），2.0m 只走 1.0m
- v2 先南让到 y≈-2.0 再向东 → 第四段被 `pillar_a(1.8,1.5)` 挡住，4.6m 只走 0.32m
- v3 北段抬到 y≈2.0 从柱子上方过 → 全程 6 段无碰撞，未知区域 7.8%→2.5%

## 六个环境坑（不是项目代码问题）

1. **托管 python 3.13 抢占 PATH** → ROS 的 Python 节点全废（`No module named 'numpy'`）。
2. **`/dev/shm` 仅 10MB** → FastDDS SHM 传输失败，`ros2 topic list` 跨终端全空，需强制 UDP。
3. **Bash 沙箱每次调用结束回收子进程** → `nohup`/`setsid`/`disown` 都留不住，
   多进程流程必须写在**一个脚本里一次跑完**。
4. **`ros2 topic pub --once /initialpose` 送不到 AMCL** → 必须持续发 3 秒。
5. **必须先给 AMCL 初值 Nav2 才会激活** → `planner_server` 的 costmap 在 activate
   阶段等 `map→odom`，卡住则 `bt_navigator` 永远 inactive。
   注意此前的「导航成功」是残缺链路（只有 `controller_server` active），不算通过。
6. **`SpawnEntity` 的 `initial_pose` 必须显式给坐标** → SDF 的 `<pose>` 与
   `initial_pose` 会叠加，全零会让箱子落在世界原点与车重叠，
   表现为「spawn success=True 但激光看不到、Nav2 不标记」。

## 非致命现象（出现属正常，别误判）

| 现象 | 根因 |
|---|---|
| `gzclient` 崩溃 `failed to load driver: iris` | 无 `/dev/dri`，llvmpipe 软渲染；用 `gui:=false` 跑数据流，物理不受影响 |
| RViz 退出时段错误 | 段错误发生在 `signal_handler` 之后（OGRE 释放资源），运行期与截图正常 |
| RViz `indexed_8bit_image` GLSL 警告 | llvmpipe 下地图图层的着色器兼容提示，渲染结果正确 |

## RViz 视图调整

`navbot_nav.rviz` / `navbot_slam.rviz` 的 `TopDownOrtho Scale` 由 **60 → 150**，
原来 6m 房间只占视野 360px，看不清代价地图的膨胀层。

---

## 2026-11-10 参考项目评估（ros2-gazebo-robot-lab）

详见 `docs/S21_参考项目评估_ros2-gazebo-robot-lab.md`。

**结论：代码不能抄，方法可抄。**

该项目的技术栈与本项目完全不同（ROS 2 Lyrical + Gazebo Sim 10.5 + Docker +
2m×1.9m 大方车 + 24×20m 场景），**SDF / launch / 话题名 / 帧名 / Nav2 参数值全部不兼容**。

值得借鉴的三件事：

1. **`safety_contract` 模式** —— 阈值只定义一次，规划器与急停共用，纯函数零 ROS 依赖。
   已落地为 `docs/SAFETY_LIMITS.md` + `tools/verify/check_limits.py`（7 项判据）。
2. **实验证据归档** —— `docs/experiments/YYYY-MM-DD/{README,evidence,images}`，
   summary.json 里带 `stop_verification`（停车后速度确认）与 `caveat`（数据边界）。
3. **结论主动划界** —— 每条结论后跟一句"这不能说明什么"。

### 顺带纠正了两处旧记录

- 本项目**有 GPU**（Intel RPL-P，OpenGL 4.6），"无 /dev/dri"是错误的；
  `gzclient` 崩溃的真因是 `env.sh` 没 source `/usr/share/gazebo/setup.sh`
  导致 `GAZEBO_RESOURCE_PATH` 为空 → OGRE 找不到 shader 库。已修。
- 第 3 条坑需补充：2026-11-10 复测确认「跨调用回收」**仍然成立**
  （11-09 曾误判为能存活）。且一条调用跑完整链路会撞 120s 超时被 SIGTERM，
  建图中途被杀导致地图未知区 18.4%（完整图 2.5%）。
  ⇒ 助手只做单次 <100s 的短任务，长流程交 `live.sh` 在用户终端跑。

---

## 2026-11-10 route_loop 回不到起点：定位为路线设计问题并修掉

用户跑 `./drive.py loop` 后反馈「小车最后停止的位置离原来的位置差距也太大了吧」。
实测 `odom = (-2.616, -0.195)`，`map→odom` 平移仅 `[0.006, 0.012, -0.000]`。

### 结论

**车走得极准，路线本身不闭合。** 旧 route_loop 东行 2.4 / 西行 5.0，
x 方向净差 −2.6 m；理想几何终点就是 `(-2.600, -0.200)`，与实测只差 1.6 cm。

### 改了三个地方

1. `turn()`：tol 0.03→0.004 rad，stop 后加 `_pump(0.25)` 再读一次 yaw。
   单次残差 −0.96°→−0.05°，7 次累计 +4.81°→+0.23°。
2. `route_loop()`：改成对称矩形（南 y=−2.05 / 北 y=+2.05 / 东 x=+2.50 / 西 x=−2.50），
   位置精确闭合，末尾 `_finish_at_origin` 掰正朝向 + 位置闭环。
3. `route_survey()`：老版净位移 0.688 m、朝向 180°；改为正反各绕一圈同一矩形。

抽出公共函数 `_drive_rectangle(d, reverse)` 与 `_finish_at_origin(d)`。

### 新增验证工具

- `tools/verify/sim_fullchain/loop_geom.py` —— 逐段位置/朝向 + `--check` 航线净空 + `--real` 叠实测损耗
- `tools/verify/sim_fullchain/turn_sim.py` —— 复现 turn 收敛行为，对比新旧残差
- `tools/verify/sim_fullchain/selftest_drive.py` —— 假 ROS + 一阶惯性模型，**26 项全 PASS**

### 复盘文档

`docs/S22_复盘_route_loop闭环失败.md`

### 待真车确认

跑 `./drive.py loop`，看日志末尾「开环收尾位姿 ... 距原点 d m」，判据 d < 0.05。

---

## 2026-11-10 建图质量量化：判定「地图是准的，不改参数」

闭环跑通后我看 RViz 截图说「墙外有绿色毛边、SLAM 对齐差」，准备调
`minimum_travel_distance` / `minimum_travel_heading`。**实测否掉了这个判断。**

### 实测（map_quality.py）

```
未知区 2.2%（判据 <15%）  已知区 97.8%
最大墙体偏差 0.042 m < 1 个栅格（0.05 m）
verdict: PASS
```

绿色点是 `LaserScan` 实时渲染，占据格是历史累积，两者叠加本来看起来就像毛边。
**不是问题。**

### 三次测量方法才得到可信数字（三次都是基准错，不是地图错）

1. 沿法向 break 在第一个占据格 → 系统偏内（内表面先被填）
2. 取首尾中点 → 更差（外墙外侧无占据格，中点被拉向内侧半墙厚）
3. ✅ 取 `min(occ, key=abs)` + **真值改用内表面**（中心线 ∓ 0.05）

> 核心教训：激光只能测到它能看到的那一面。拿墙心当真值，测什么都是错的。

### 顺手排除：雷达前偏 4cm 不是问题来源

`base_footprint→laser_link = [0.040, 0, 0.100]`。slam_toolbox 用 TF 投影，
不会整体平移。且四墙偏差同号同量级 = 栅格量化特征，不是单方向平移。

### 新增

- `tools/verify/sim_fullchain/map_quality.py` —— 6 条墙 vs 真值，3 条判据，
  已做故障注入验证（真值故意错 0.3m → 正确报 FAIL）
- `docs/S23_建图质量量化.md`

### 待办（低优先级，等提速前再动）

`minimum_time_interval: 0.5` 而雷达 10Hz → 丢弃 80% 帧。当前 0.15 m/s 无影响；
跑 Nav2 或提速到 0.3+ m/s 前应改 0.2。**现在没有证据说它影响建图，不动。**

---

## 2026-11-10 Nav2 导航验证准备

### 地图更新（旧图是修复前建的）

- 旧 `navbot_room.pgm` 建于 11-09 21:22，是**不闭合路线**时期建的
- 用 `map_quality.py --save` 存当前图并替换：未知区 2.5%→**2.2%**，墙偏差 0.045→0.042
- 旧图备份 `maps/.bak/navbot_room_20261110_before.*`
- **回读验证**过 PGM 行序（写完按 yaml 还原世界坐标重测墙体，全 OK）

### 保存地图的三个格式坑（已修在 map_quality.py）

1. `image:` 写绝对路径 → 改相对文件名（装到 install/share 会失效）
2. `free_thresh: 0.196` 与仓库既有 0.25 不一致 → 统一 0.25
3. 先复制 yaml 再删旧 pgm → yaml 指向已删文件。**改引用要在删文件之前**

### 目标点净空预检（新发现两个陷阱点）

- `(-2.20,-1.50)` 净空 0.150（pillar_b）—— **不可达**
- `(0.90, 0.50)` 净空 0.100（partition_mid）—— **不可达**
- ~~窄道只能「穿过」测~~ ← **这条是错的，见下面 S25 的纠正**
- 安全替代：`(0.70,-0.10)` 净空 0.450，正对通道口

### 新增 nav_patrol.py（7 点序列）

~~`东南角→南走廊→西北角→北走廊→东北角→窄道口南→窄道后北`，后两点强制穿 0.6m 窄道。~~
**↑ 整条作废。那条 0.6m 窄道不存在，见 S25。**
判据：SUCCEEDED + 误差<0.25m + 用时<120s。`parse_result` 做了 4 组输入自测。

### 待手动执行（助手起不了 GUI）

当前仍是建图模式（ROS 图里只有 slam_toolbox）。切换：
```
Ctrl+C 停掉 live.sh
bash ~/navbot_ws/tools/verify/sim_fullchain/live.sh nav
```
然后 `python3 nav_patrol.py`。

文档：`docs/S24_Nav2导航验证准备.md`


---

## 2026-11-10 傍晚 纠正：房间几何全算错了（见 docs/S25）

准备跑 Nav2 巡检时重新核净空，发现上一节里关于「窄道」「0.6m 通道」的整套说法
**全是我凭空想的**。连着错了三次，每次都是同一个病根。

### 真实拓扑（地图逐行扫描反推，不再靠记忆）

```
y=+0.55 ─── partition_mid ───         x∈[-1.78,+0.82] y≈+0.50
              ↑ 缝N 0.450m
y=+0.00    ┌──┐ partition_short       x∈[+1.12,+1.32] y∈[-1.50,0.00]
           │  │  ← 缝E 0.300m 死路
y=-1.45 ───┤  ├── partition_stub      x∈[-2.03,-1.73] ← 之前完全不知道有这块
y=-1.50    └──┘
              ↑ 缝S 0.950m
```

| 通道 | 缝宽 | 物理 | 非致命带 | 零代价带 | 结论 |
|---|---|---|---|---|---|
| 缝 N | 0.450 | 过（余5cm） | 0.050 | 0.000 | 能规划但代价高，会绕 |
| 缝 E | 0.300 | **不过** | 0.000 | 0.000 | 死路 |
| 缝 S | 0.950 | 过 | 0.549 | 0.449 | 宽敞 |
| 缝 W | 1.170 | 过 | 0.770 | 0.669 | 宽敞 |

### 三次错误

1. 设 `(0.70,-0.10)→(0.70,1.30)` 当"0.6m 窄道"——净空 0.45/0.75，全在开阔区
2. 设 `(0.30,-2.10)→(0.30,0.10)` 当"窄道E"——x=0.30 处距南墙 2.35m，也是开阔地
3. 断言"东走廊 1.7m 是唯一南北通路"——西侧也有 1.17m，不唯一

**根因**：`room_geom.py` 的 `PARTITIONS` 常量是我早前凭印象写的，
却被 `loop_geom.py` / `map_quality.py` / `drive.py` 多处引用，
其中两个还给它写了"验证通过"的自测。**自测只能证明代码符合模型，不能证明模型符合现实。**

### 纠正后

- `room_geom.py` 的 `PARTITIONS` 改成地图实测值（三块板，含漏掉的 stub）
- 自测期望值全部注明来源；凭直觉写数字的部分改成验**恒真性质**（单调性/上界）
- `nav_patrol.py` 路线改为 `东南角→南走廊→西南角→西侧通道→西北角→北走廊→东北角→东侧通道`
- 新增 `--plan` 离线 A* 预演：**不起仿真就能验整条序列**

### 离线预演结果

```
8 点全 PASS，总路程 19.76m，直线累加 18.72m，绕行系数 1.06
最小净空 0.252m（> 车半径 0.20）
```

绕行系数 1.06 说明路径很顺。结论：**当前参数不用动**，
之前担心的「inflation 0.25 会让 N 缝封死导致北区不可达」不成立——
三条路都通，规划器会自己选代价低的（西侧）。

### 待手动执行

仍是建图模式，需要切到导航模式（助手起不了 GUI）：

```bash
# Ctrl+C 停掉当前 live.sh
bash ~/navbot_ws/tools/verify/sim_fullchain/live.sh nav
```

另一个终端：
```bash
source ~/navbot_ws/tools/verify/sim_fullchain/env.sh
python3 nav_patrol.py
```

文档：`docs/S25_复盘_房间几何算错三次.md`

---

## S26 地图质检方法修正（2026-11-10）

`map_quality.py` 原来 6/7 OK、`partition_stub` 一条 FAIL（+0.175 m）。
修完后 **7/7 全 OK，最大偏差 0.042 m，未知区 2.2%**。

**地图一直没问题，是质检方法错了三次**（详见 `docs/S26_地图质检的三个方法坑.md`）：

| # | 错在哪 | 真相 | 修法 |
|---|--------|------|------|
| 1 | 用「离房间中心最近的占据格」量独立小块 | 那条判据只对外墙成立（一侧可见）；独立块四侧都可见，「最近」锁到上表面 | 新增 `measure_box()`，穷举包围盒取中心 |
| 2 | `--offline` 读 PGM 用了默认原点 `(-3.1,-5.49)` | 实际 yaml 原点是 `(-2.983,-2.495)`，y 差 3 m → 整图平移 | 新增 `_read_yaml()`，原点必从 yaml 读 |
| 3 | ★ 把 `partition_stub` 当 0.35×0.35 方板 | world 文件里**根本没有**这个模型，是 `pillar_b`：**半径 0.15 圆柱** | 改 `PILLAR_A/PILLAR_B` 元数据 + `pillar_passage()` |

坑 3 的连带影响：
- 圆柱在 5cm 栅格上外接框必然是 `2r + 1格 = 0.35`，**不是误差，是栅格化必然**
- 拿 0.35 当矩形算净空会**系统性把柱子算粗半格**，0.45 m 的缝上足以改变能不能过
- `pillar_a`（1.8, 1.5, r=0.15）在**北区**，之前从未进过任何自测，现已纳入

### 顺带修掉的架构隐患

`map_quality.py` 的 `WALLS` 原来是手抄的第二份真值，和 `room_geom.PARTITIONS` 各改各的。
S25 改了 `PARTITIONS` 但 `map_quality.py` 没同步 → 一直 FAIL 却以为「真值已修好」。

现在改成 `_derive_truth()` 从 `room_geom` 派生。规矩：**算错就撞墙的数字只允许一个来源。**

同时修掉分类判据：原 `min(dx,dy) <= 0.40` 会把 `0.25×1.80` 的 `partition_short`
误判成方块，改为 `max(dx,dy) <= 0.45`（两个方向都得短）。

### 回归结果

```
room_geom.py  --selftest   全部 PASS（自测期望值改为恒等式 + 形状校验）
nav_patrol.py --selftest   全部 PASS
nav_patrol.py --dry        8/8 可达，预检全部通过
nav_patrol.py --plan       总 19.97 m，绕行系数 1.07，最小净空 0.257 m
map_quality.py --offline   7/7 全 OK
```

### 新增能力

```bash
# 不启 ROS 也能复算地图质量（读 yaml 拿原点）
python3 map_quality.py --offline ~/navbot_ws/src/navbot_bringup/maps/navbot_room.pgm
```

---

## S27 8 点巡检 2 个 FAIL 的根因与修复（2026-11-10）

用户反馈原话：**「连续几次了，目标点在墙后面，小车遇到墙就卡在那里了」**

巡检结果：PASS 6 / FAIL 2。

| 目标点 | 结果 | 误差 | 用时 | 判定 |
|---|---|---|---|---|
| 东南角 | ABORTED | 1.499 m | 179.1 s | FAIL |
| 东北角 | SUCCEEDED | 0.256 m | 13.3 s | FAIL（超判据 0.006） |

### 根因：四层门槛，卡在第③④层

| 层 | 判据 | 卡点 |
|---|---|---|
| ① 物理能过 | `缝宽 > 2R` (0.400) | 缝E 0.300 → 死路，无解 |
| ② 能规划 | 非致命带 > 0 | 缝N 0.450 ✓ |
| ③ **愿走** | `缝宽 > 2·inflation` | 旧 0.50 → **缝N 整条带代价** |
| ④ **敢停** | 到达球内有合法车心 | 东北角 `xy_goal_tolerance` 0.15 偏松 |

**③ 的机制**（`live_nav.log` 705~922 行）：
`Passing new path to controller` 每秒都在打 → 规划器没失败；
`Failed to make progress` 每 10 秒一轮、共 17 轮 → DWB 算不出可执行速度。
缝N 整条带代价时，沿路径走被 `BaseObstacle` 罚、离开路径被 `PathAlign` 罚，
最优解就是 **v=0**。所以是「停住」不是「撞墙」。

**④ 的机制**：`xy_goal_tolerance` 沿路径量距离，但车心必须落在 free 格。
目标贴墙时容差球有一块是非法区，车在球外就停了。⇒ **收紧到 0.12 反而误差更小**。

### 改动

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | `inflation_radius` 0.25→**0.22**（两处）、`required_movement_radius` 0.5→**0.15**、`xy_goal_tolerance` 0.15→**0.12**（两处）、`allow_unknown` true→**false** |
| `tools/verify/check_limits.py` | `EXPECTED_INFLATION` 0.25→0.22 |
| `room_geom.py` | `INFLATION` 0.22；新增 `reachable_center()`；自测[2][6]断言改为恒真性质、新增[7]可达域 |
| `nav_patrol.py` | `--plan` 新增「链路累计漂移」「可达域」两列 |
| `docs/SAFETY_LIMITS.md` | 补「缝N 才是真瓶颈」，0.25→0.22 推导 |
| `docs/S27_导航卡住的四层病因.md` | 新增，含完整推导与三个自查错误 |

### 回归结果

```
check_limits.py            7/7 PASS
room_geom.py  --selftest   全部 PASS（含新增 [7] 车心可达域）
nav_patrol.py --selftest   全部 PASS
nav_patrol.py --dry        8/8 可达，预检通过
nav_patrol.py --plan       总 19.86 m，最小净空 0.225 m，全序列 PASS
colcon build navbot_bringup  0.61s（symlink-install，参数改动即时生效）
```

> ★ 尚未在仿真里重跑巡检验证。以上全部是离线判据。

### 三个自查错误（已写进 S27 文档）

1. 以为缝E 能靠降 inflation 救回来 —— 物理差 100 mm，降 0 也没用。
2. 自测把「当前值」当「期望值」写死 —— 改参数后自测 FAIL 的是断言不是代码。
3. 可达域断言写错两次：把「最远距离」当「面积」；忘看缝宽只有 0.300。
   最终改成「取缝最窄那一列做单点探针」才稳定。

---

## S28 原地转向时 DWB 瞎了（2026-11-10）

用户反馈原话：**「还是不行」** —— S27 改完 `inflation_radius` 0.22 之后，东南角仍然 ABORTED。

| 目标点 | 结果 | 误差 | 用时 | 备注 |
|---|---|---|---|---|
| 东南角 (2.20, −1.50) | **ABORTED** | 1.499 m | **177.0 s** | 车停在 (0.70, −1.58)，第 1 点 |
| 第 2~5 点 | SUCCEEDED | — | — | 正常 |

### 先证明「物理上完全能走」

- 车停在 (0.70, −1.58)，原地转所需净空 = 车半径 0.192 m，实测前方净空 **0.417 m** → 够转
- 通往目标的缝 S 中心线 (0.70, −2.00)，costmap 代价 **0**（S27 调的 inflation 0.22 生效了）
- ⇒ 不是「过不去」，是「不肯转」

### 根因：v=0 时四个 position critic 全部退化

`PathAlign` / `PathDist` / `GoalAlign` / `GoalDist` 算的都是**轨迹末端点**到路径/目标的距离。
原地转（v=0, w≠0）时末端点恒在车心 → **转左转右同分**，critic 无法区分。

而唯一管朝向的 `RotateToGoalCritic` 源码里写着：

```cpp
if ((goal_x_ - pose.x_)² + (goal_y_ - pose.y_)² > xy_goal_tolerance_²)
    return 0.0;   // ← 远离目标时完全不参与打分
```

车距目标 1.50 m ≫ `xy_goal_tolerance` 0.12 → **权重 0**。

于是 DWB 面临的选择是：往前走（被 `BaseObstacle` 罚）vs 停住（0 分）→ 选 0 分。
`SimpleProgressChecker` 又只看位移（原地转位移恒 0），10 s 就判没进展 → spin → wait → backup 全试过，最后 ABORTED。

### 修法：改结构，不是调权重

| 方案 | 做法 | 结论 |
|---|---|---|
| 调权重 | 把 `PathAlign` 加大 | ✗ 末端点仍退化，加大只是放大噪声 |
| 改退化的量 | `forward_point_distance` 0.1→0.25 | △ 有改善但只到 0.5 m 内有效 |
| **改结构** | **外层套 `RotationShimController`** | ✓ 先原地转到朝路径切线，再交 DWB |

`RotationShimController`（`libnav2_rotation_shim_controller.so`）检测车头与路径切线夹角 > `angular_dist_threshold` 时先原地转，转完交出内层。

### 改动

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | 控制器改双层：`FollowPath` = RotationShim / `FollowPathDWB` = DWB |
| `nav2_params.yaml` | `primary_controller` / `angular_dist_threshold` 0.35 / `rotate_to_heading_angular_vel` 0.8 / `max_angular_accel` 2.0 / `rotate_to_heading_when_stopped` **true** |
| `nav2_params.yaml` | `movement_time_allowance` **10→25**（shim 原地转位移为 0，10 s 会被反复踩线） |
| `nav2_params.yaml` | `PathAlign/GoalAlign.forward_point_distance` 0.1→**0.25**、新增 `Twirling.scale: 0.01`、`BaseObstacle.scale` 0.02→**0.1** |
| `tools/verify/check_controller.py` | 新建，9 项检查 + 4 个故障注入自证 |
| `tools/verify/sim_fullchain/live.sh` | `nav` 模式加「[0/3] 参数闸门」 |

> ★ **命名坑**：外层必须叫 `FollowPath`（`controller_plugins` 里的名字），内层必须叫别的（`FollowPathDWB`）。
> 若内层也叫 `FollowPath`，shim 会把自己当内层 → 无限递归。

### 参数名不是猜的

用 `strings libnav2_rotation_shim_controller.so | grep -iE "angular_dist|rotate_to_heading|primary_controller|max_angular|closed_loop"` 提取真实参数名。
结果：`angular_dist_threshold` / `closed_loop` / `max_angular_accel` / `primary_controller` / `rotate_to_heading_angular_vel` / `simulate_ahead_time` 确认存在；
**`rotate_to_heading_min_angle` 不存在**（我一开始凭印象写的，已删）。

### 回归结果

```
check_controller.py           PASS=9  FAIL=0
check_limits.py               PASS=7  FAIL=0
room_geom.py --selftest       全部 PASS
nav_patrol.py --selftest      全部 PASS
nav_patrol.py --dry           8/8 可达
nav_patrol.py --plan          全序列 PASS
bash -n live.sh               通过
colcon build navbot_bringup   0.56s
```

4 个故障注入全部被抓到：

| 注入 | 检出 |
|---|---|
| `primary_controller` 指向自己 | FAIL 会无限递归 |
| `primary_controller` 写错名 | FAIL shim 起不来 |
| `rotate_to_heading_when_stopped: false` | FAIL 差速车窄处会边转边走 |
| `movement_time_allowance: 10.0` | FAIL 一转向就被判没进展 |

> ★ **`check_controller.py` 的第一版 `MIN_ALLOWANCE_FOR_ROTATE` 我拍了个 8.0，故障注入立刻打脸**——
> 把 allowance 从 25 改回 10，检查仍全 PASS，判据是摆设。
> 按梯形速度曲线重算：转 70° = 加速 0.40 + 匀速 1.13 + 减速 0.40 = 1.93 s，
> + DWB 起步 1~2 s + planner 降频抖动 2~3 s + 一次恢复 7 s ⇒ 12.9 s，取下限 **15.0**。

> ★ 尚未在仿真里实跑验证。这次改的是控制器结构，风险高于调参数，建议**先单点验证**：
> `python3 nav_patrol.py --one 2.20 -1.50`

### 教训

1. **「能规划」≠「能执行」** —— planner 每次都在给新路径，DWB 就是不走。
2. **三种「不动」要分清**：停住（v=0 有解但被罚）/ 撞墙（无合法轨迹）/ 原地打转（oscillation）。判据看日志有没有 `Failed to make progress`、`costmap` 里车心代价。
3. **改结构 > 调权重** —— 退化的是「量本身」，加权只是放大噪声。
4. **判据没被验证过，就等于没有判据** —— 和 S27「自测把当前值当期望值」是同一类错误。

---

## S29 检查脚本全绿，仿真却起不来（2026-11-10 第二轮）

S28 改完 `RotationShimController`，离线 9/9 全通，一跑仿真：

```
NOT_ACCEPTED 误差 2.667 m  用时 0.8s  FAIL
```

**`NOT_ACCEPTED` ≠ S27 的 `ABORTED`**：后者是「走到了走不下去」，前者是「目标根本没被受理」。

### 日志一句话定位

```
[FATAL] Failed to create internal controller for rotation shimming.
        Exception: the class FollowPathDWB with base class type
        nav2_core::Controller does not exist.
        Declared types are dwb_core::DWBLocalPlanner ...
[INFO]  Controller Server has FollowPath controllers available.   ← 在骗人
[INFO]  Activating controller: FollowPath of type ...RotationShimController
[ERROR] process has died [exit code -11]
```

`configure()` catch 到异常只 `return`，把 `primary_controller_` 留成 `nullptr`；
生命周期管理器照样 activate；`activate()` 第一句 `primary_controller_->activate()`
→ 空指针解引用 → **SIGSEGV**。

### 根因一：`primary_controller` 是插件类名，不是参数块名

```cpp
primary_controller = node->get_parameter(plugin_name_ + ".primary_controller").as_string();
primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);  // 按类名
primary_controller_->configure(parent, name, tf, costmap_ros);              // name = 外层名
```

| 参数 | 正确值 |
|---|---|
| `primary_controller` | `"dwb_core::DWBLocalPlanner"` |
| 内层参数前缀 | `"FollowPath."`（shim 自己的 plugin_name） |

### 根因二：`FollowPathDWB:` 块从头到尾没人读

内层 `configure(parent, name, ...)` 的 `name` 是 shim 自己的 `plugin_name`。
另开的块**不报错**，DWB 用默认值（`max_vel_x` 0.26 vs 本车 0.22）静默跑飞。

### 根因三：`rotate_to_heading_when_stopped` 是我编的

`.so` strings 只有 6 个 shim 参数，源码 `configure()` 里没有这个。
原地转由 `computeRotateToHeadingCommand()` 保证：只填 `angular.z`，
`linear` 保持零初始化 ⇒ 天然原地转。该开关是 Jazzy+ 才有。

### 更严重：检查脚本把错的当成了对的

`check_controller.py` 原判据是「`cs` 里有没有这个 key」——
`cs` 是我自己写的字典，自己的答案核对题目，永远是绿的。

**新判据（问系统，不问自己）**：

```python
classes = find_plugin_classes('/opt/ros/humble/lib')   # strings 扫 .so 抠类名
if inner_cls in classes: ok(...) else: ng('类名不存在 → FATAL + SIGSEGV')
```

**新增自证**：`tools/verify/inject_faults.py`，复制 yaml → 注入错误 → 断言检查必须红。

### 改动

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | `primary_controller` → `"dwb_core::DWBLocalPlanner"` |
| `nav2_params.yaml` | DWB 的 37 项参数从 `FollowPathDWB:` **移进 `FollowPath:`**（47 键） |
| `nav2_params.yaml` | 删 `rotate_to_heading_when_stopped`；新增 `simulate_ahead_time` 1.0 / `forward_sampling_distance` 0.5 / `angular_disengage_threshold` 0.5 |
| `tools/verify/check_controller.py` | 重写：`.so` 类名扫描 + 游离块检测 + 独立子插件排除 |
| `tools/verify/inject_faults.py` | 新建，6 种故障注入 |
| `tools/verify/sim_fullchain/live.sh` | 闸门加第三段「控制器判据自证」 |
| `docs/S29_检查脚本全绿仿真却起不来.md` | 新建，十节 |

### 回归

```
check_controller.py     PASS=11  FAIL=0
inject_faults.py        抓到 6 / 漏掉 0（前 3 项就是这次的三个真实错误）
check_limits.py         PASS=7   FAIL=0
nav_patrol.py --dry     8/8 可达，全序列 PASS
bash -n live.sh         通过
colcon build            0.67s（install 里是符号链接）
```

### 改 yaml 结构踩的三个坑

| 错误 | 症状 | 原因 |
|---|---|---|
| ① 缩进剥多了 | 参数掉出 `FollowPath` | `l[2:]` 让 6→4 格 |
| ② 块尾判定错 | 参数挂进 `general_goal_checker`（40 键） | 该块本身也是 4 格缩进，与 `FollowPath` 同级 |
| ③ `/tmp` 不共享 | 中间文件下次调用就没了 | 本机 Bash 老问题 |

**正确姿势**：一次 heredoc 做完；用**锚点字符串**定边界不看缩进；每步用
`yaml.safe_load` 打印**每个块的键数**（`general_goal_checker 40 键` 就是出事了）。

> ★ 仍未实跑验证。下一步先 `ros2 lifecycle get /controller_server` 看是不是 active，
> 再 `--one 2.20 -1.50`。

---

## S30 东南角跑通了（2026-11-10 第三轮，实跑验证）

`primary_controller` 改成类名 + DWB 参数归位到 `FollowPath:` 之后，**单点实跑通过**。

```
$ python3 tools/verify/sim_fullchain/nav_patrol.py --one 2.20 -1.50
起点 AMCL 位姿 (0.013, 0.008)
→ 指定点   目标 (2.20, -1.50)
SUCCEEDED  误差 0.111 m  用时 35.0s  PASS
合计 1 点，PASS 1，FAIL 0
```

### 三轮对比

| | S27（inflation 0.22） | S28（shim，配置错） | S30（shim，配置对） |
|---|---|---|---|
| 结果 | `ABORTED` | `NOT_ACCEPTED` | **`SUCCEEDED` PASS** |
| 用时 | 177.0 s | 0.8 s（进程崩了） | **35.0 s** |
| 误差 | 1.499 m | 2.667 m | **0.111 m** |

### 三个关键指标全部归零

| 指标 | S27 | S30 |
|---|---|---|
| 恢复动作 `spin`/`wait`/`backup` | 3 次 | **0** |
| `Failed to make progress` | 17 轮 | **0** |
| `Planner loop missed its desired rate` | 7 次 | **0** |

> ★ 这三项比「结果 PASS」更有说服力：
> 它说明车不是「侥幸绕过去了」，而是**全程没被判定卡住过**。

### 内层控制器确认建起来了

```
[controller_server]: Created internal controller for rotation shimming:
                     FollowPath of type dwb_core::DWBLocalPlanner
```

且 `grep -c "process has died"` = **0**（S29 那轮是 1，SIGSEGV）。

### 顺带发现：planner 降频也是被连带治好的

S27 日志里 7 次 `Planner loop missed its desired rate of 20Hz. Current loop rate is 1.6667 Hz`。
本轮 0 次。原因不是改了 planner 参数，而是**车不再长时间原地打转**——
每次卡住时 planner 都在高频重规划，CPU 吃满才降频。卡住消失了，降频自然消失。

> 这是个值得记住的因果方向：**「planner 降频」往往是症状，不是病**。
> 病在控制器不肯执行，planer 只是在空转。

### 新增工具：watch_cmdvel.py

以前判断「shim 有没有真的原地转」只能靠间接推断（shim 那条 `rotating to heading`
是 DEBUG 级别，默认级别看不到）。新脚本旁听 `/cmd_vel_nav`，输出运动画像：

```
$ python3 tools/verify/sim_fullchain/watch_cmdvel.py
 PURE_ROTATE        1.45    26.4%  ← shim 原地转的指纹
 PURE_LINEAR        1.95    35.5%
 MIXED              1.45    26.4%  ← 边转边走，窄处要小心

★ 检测到 1 段真正的原地转（转角 > 0.15 rad ≈ 8.6°）：
       0.50s 起  转 66.5°  用时 1.45s
  ⇒ RotationShimController 确实介入了
```

分类阈值（非拍脑袋）：
- `linear <= 0.01` = `max_vel_x 0.22` 的 4.5%，低于此是数值逼近不是真走
- `angular <= 0.05` = `max_vel_theta 1.0` 的 5%
- 段长 `< 0.15s` 视为抖动，不独立成段

自测：构造「停 0.5 → 原地转 1.5s@0.8rad/s → 直行 2s → 边转边走 1.5s → 停 0.3s」，
分类与转角累计均正确（期望 68.75°，实测 66.5°，差在 0.05s 采样边界）。

### 遗留

- **8 点完整巡检尚未跑**（本轮只验了单点）
- `avoid_test.py` / `verify_avoid.sh` 的自主避障验证仍未执行
- `loop_geom.py` 第 29-30 行仍引用旧几何常量
- `slam_toolbox` 的 `minimum_time_interval: 0.5` vs 雷达 10 Hz（丢弃 80% 帧，低优先）

---

## S31 8 点全 PASS 之后的爬行问题（2026-11-10 第四轮）

`nav_patrol.py` 首次跑出 **PASS 8 / FAIL 0**：

| 目标点 | 误差 | 用时 |
|---|---|---|
| 东南角 | 0.125 m | 31.8 s |
| **南走廊中点** | 0.105 m | **67.8 s** ← 异常 |
| 西南角 | 0.076 m | 34.8 s |
| 西侧通道 | 0.117 m | 33.2 s |
| 西北角 | 0.110 m | 23.6 s |
| 北走廊中点 | 0.123 m | 14.4 s |
| 东北角 | 0.112 m | 15.0 s |
| 东侧通道 | 0.117 m | 25.7 s |

误差全部 0.076~0.125 m（判据 0.25），位置精度没问题。**问题在耗时分布。**

### 定位：59 秒爬了 2.21 米

| 时间戳 | 事件 |
|---|---|
| `3022.31` | 开始导航 `(2.13,-1.40) → (0.00,-2.00)` |
| `3081.93` | `Failed to make progress`（**已 59.6 s**） |
| `3081.95` | 中止 → **清空 local_costmap** → 重发 |
| `3088.11` | `Reached the goal`（**只用 6.2 s**） |

直线距离 2.213 m：

| 阶段 | 平均速度 | 占 max_vel_x 0.22 |
|---|---|---|
| 卡住 59.6 s | **0.0371 m/s** | 17% |
| 清 costmap 后 6.2 s | 0.357 m/s | 162% |

**同一段路快 9.6 倍。** 而 `progress_checker` 25s/0.15m 只要 6 mm/s 就达标 → 37 mm/s 永远不响。

### 几何：缝 S 北入口被膨胀封死

```
  y=-1.80  x1.5:## x1.4:## x1.3:## ...   ← 全致命
  y=-1.90  x1.5:23 x1.4:## x1.3:## ...
  y=-2.00  x1.5:.. x1.4:.. x1.3:24 ...
```

`y >= -1.90` 全封，必须退到 `y = -2.00`。几何上是**直角路线**（先南后西）。
但 A* 给的是擦边斜线：

```
直角绕行 = 2.48 m
A* 段长  = 2.41 m   ← 绕行系数仅 1.09
直线     = 2.21 m
--plan 最小净空 = 0.225 m  ← 离零代价线只差 5mm
```

沿直线采样：`(1.49,-1.58)` 净空 0.174 < 车半径 0.20；`(1.28,-1.64)` 净空 **0.000**。

### 根因：BaseObstacle 的量级压不住 PathAlign

**源码事实（`base_obstacle.cpp`）——推翻我 S28 的注释：**

```cpp
unsigned char cost = costmap_->getCost(cell_x, cell_y);
if (!isValidCost(cost))  throw IllegalTrajectoryException("Hits Obstacle");
return cost;             // 返回原始 0..252，不归一化
bool isValidCost(c) { return c != 254 && c != 253 && c != 255; }
```

⇒ **253/254 是「淘汰」不是「罚分」**；S28 注释里「253 × 0.02 = 5.06 分」是错的。

真实量级（scale=0.1）：

| 项 | 分数 |
|---|---|
| 贴墙（净空 0.210, cost 240） | 240 × 0.1 = **24.0** |
| PathAlign 偏 0.05m | **6.4** |
| PathAlign 偏 0.10m | **12.8** |

**贴墙省 24、离开 10cm 花 12.8 → 净赚 11.2，太小。DWB 选择贴着爬。**

### 排除的三个方向（都算过账）

| 方向 | 结论 |
|---|---|
| 降 `cost_scaling_factor` | **无效**。0.22−0.20=0.02m 带宽，k 从 5 降到 1 代价仅 247→252，是几何决定的 |
| 加 `inflation_radius` | **被缝N卡死**。要 ≥5cm 缓坡需 R≥0.25，但缝N 2R 上限 0.225，现 0.22 只剩 5mm |
| 开 `sum_scores` | **帮倒忙**。累加量级 ~sim_time×freq，窄缝更容易误杀安全轨迹 |

### 修法：BaseObstacle.scale 0.1 → 0.5

| | scale=0.1 | scale=0.5 |
|---|---|---|
| 贴墙（cost 240） | 24.0 | **120.0** |
| 偏 10cm 花 | 12.8 | 12.8 |
| 净收益 | +11.2 | **+107.2** |

取 0.5 不取 1.0：太高会让车在宽走廊里「贴单侧墙」。
若仍有爬行 → 上 1.0；若出现贴单侧墙 → 回退 0.3。**这是必须实跑对比的取舍。**

### 改动

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | `BaseObstacle.scale` 0.1 → **0.5**；新增 `BaseObstacle.sum_scores: false`（显式） |
| `nav2_params.yaml` | 修正 S28 的错误注释（253 不参与打分） |
| `check_controller.py` | 新增第 ⑦ 项「BaseObstacle 量级」，判据 = 净收益 ≥ 23 分 |
| `inject_faults.py` | 新增第 7 种注入：scale 退回 0.02 |
| `docs/S31_8点全PASS之后的爬行问题.md` | 新建，十节 |

### 回归

```
check_controller.py     PASS=12  FAIL=0
inject_faults.py        抓到 7 / 漏掉 0
check_limits.py         PASS=7   FAIL=0
nav_patrol.py --dry     8/8 可达，全序列 PASS
colcon build            0.56s
```

### 教训

1. **PASS 8/FAIL 0 不等于没问题** —— 要看每个点的耗时分布，慢 3 倍就是有故事。
2. **`movement_time_allowance` 是双职责参数** —— 同时管「原地转」和「低速爬行」。拉宽救了前者、放过后者。正解是让爬行不发生，不是收紧判据（收紧会误杀 shim）。
3. **253/254 是淘汰不是罚分**，`BaseObstacle` 用 0..252 且不归一化。
4. **先算账再改参数** —— 三个方向排除后才轮到 scale，不是一上来就调 critic。
5. **参数改动必须带反向判据** —— 提高 scale 的副作用是「贴单侧墙」，不观察副作用等于只验了收益。
6. **调用别人函数前先看签名** —— 我 `rg.inflate_cost(x, y)` 差点追一个不存在的「目标点在致命格」问题，实际第二参是 `radius`。

> ★ 尚未实跑。要同时看两件事：南走廊是否 <30s、宽走廊是否开始贴单侧墙。

---

# S32 —— 「从上面能过去，从下面就是一直卡在这里」

> 完整分析见 `docs/S32_从下面过不去.md`。

## 现象

用户贴 RViz 截图 + 一句话：「**从上面能过去，从下面就是一直卡在这里**」。

车停在 `(0.97, -1.81)`，`partition_short` 南端偏西。
日志：5 次 `Failed to make progress` → 清局部图 → planner 连续 5 次
`failed to create plan` → spin / backup 全部 `Collision Ahead` → `Goal failed`。

## 定位（一行代码定死）

`room_geom.inflate_cost` 代进车实际坐标：

```
y=-1.81  非致命段: [0.60, 0.91], [1.52, 2.75]
(0.97, -1.81) 净空 0.148 → cost 253（致命）
```

**车站在 253 的格子上**。任何轨迹末端落在这里都会被
`BaseObstacleCritic::scorePose()` 直接 `throw IllegalTrajectoryException` 淘汰
→ DWB 输出 0 → 恢复行为也因四周全是 253 而全部 `Collision Ahead`。
**三种恢复全失败 = 空间问题，不是控制器问题。**

## 根因：车被当成 0.20 的圆

| 模型 | 车宽占位 | 缝S(0.655) 车心合法带 |
|---|---|---|
| 圆 r=0.20（原配置） | 0.40 | **0.255 m** |
| 矩形 0.30×0.24 | 0.24 | **0.415 m** |

外接圆 0.192 vs 车宽一半 0.12 → **凭空多算 7.2 cm**，两侧共 14.4 cm。
缝 S 只剩 0.215 m 零代价带，而车实际只需 0.24 m 总宽 ——
**物理能过，代价图判死**。

「从上面能过」因为板东侧 1.63 m 宽，扣膨胀还有 1.19 m 零代价带；
「从下面过不去」因为南缝只剩 0.215 m 且要在 0.455 m 纵向空间里转 90°。
**两个方向可通行性差 5 倍。**

## 改动

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | `local_costmap` / `global_costmap` / `controller_server` 三处 `robot_radius: 0.20` → **`footprint` 矩形**（与 urdf `box size="0.30 0.24 0.06"` 一致） |
| `nav2_params.yaml` | `inflation_radius` 0.22 → **0.16**（= 车宽一半 0.12 + 4 cm 缓冲） |
| `check_limits.py` | 新增 ②b 节：footprint 与 robot_radius 互斥检查、尺寸对 urdf、inflation 覆盖车宽一半；膨胀过头升级为**硬 FAIL** |
| `room_geom.py` | `INFLATION` 0.22 → 0.16；新增第 [5] 节「**车体占位**判据」（8 点采样 `body_min_clear`）；修 3 条失效断言 |
| `check_controller.py` | 第 ⑦ 项代价基准 240 → 180（inflation 变小后贴墙 cost 变了） |
| `inject_faults.py` | 新增第 8/9 种注入；**改为两个 checker 都跑**（原来只跑 controller，costmap 类故障抓不到） |
| `diag_seamS.py` | 新建，缝 S 几何诊断 |
| `docs/S32_从下面过不去.md` | 新建，七节 |

## 回归（全部离线）

```
check_limits.py              PASS=10  FAIL=0
check_controller.py          PASS=12  FAIL=0
inject_faults.py             抓到 9 / 漏掉 0
room_geom.py --selftest      全部 PASS
```

## 教训

1. **「能过」和「代价图允许它过」是两个判据** —— 前者看车宽，后者看 `缝宽 − 2×模型半径`。
2. **判据必须算真实对象** —— 车心是点，车是矩形。开阔区没差别，窄缝里差 7.2 cm。
3. **用户的方向性描述胜过十次日志分析** —— 「上面能过、下面不行」直接把范围收窄到「几何南北不对称」。
4. **恢复行为全失败 = 空间问题** —— 这时不该调 critic 权重，该算「车周围还有没有合法格子」。
5. **换车体模型必须同时降 inflation** —— 只换 footprint 不降膨胀，缝还是过不去，且更难查（YAML 看着已经改对了）。

> ★ 尚未实跑。验收：8 点 PASS 8、南走廊 < 30s、`Failed to make progress` 计数 0。

---

# S33 —— 巡检脚本自己把目标弄丢了（2026-11-10 第五轮）

## 现象

S32 改完车体模型后跑 8 点巡检，得到：

```
合计 8 点，PASS 7，FAIL 1
西侧通道  [绕行]  NOT_ACCEPTED  误差 2.033 m  用时 240.5s  2.03m  0.008 m/s  FAIL 慢
```

`NOT_ACCEPTED`（不是 `ABORTED`）+ 240.5 s + 0.008 m/s，看起来像「车在爬」。但三个数字互相矛盾：
如果真在爬，Nav2 该有轨迹；如果没轨迹，240.5 s 的「用时」是哪来的？

## 定位（一条 grep 定死）

```
$ grep "Begin navigating" live_nav.log
→ from (-2.39, -2.03) to (-2.50, 1.90)     ← 西北角
→ from (-2.39,  1.88) to (0.00,  1.90)     ← 北走廊
```

**`to (-2.50, 0.00)` 一次都没出现。** Nav2 从没收到「西侧通道」这个目标。

于是 `240.5 s` 只能是脚本自己的超时：
```python
timeout 240 ros2 action send_goal ...      # ← 240.5 = 这个 timeout 触发
return 'NOT_ACCEPTED'                       # ← parse_result 的兜底分支
```
而 `2.03 m` 是脚本用「位姿差值」算的，`0.008 m/s` = 2.03/240.5，**纯属算出来的假象**。

旁边还有一条当时被忽略的强证据（失败前 4 秒）：
```
[bt_navigator.rclcpp_action]: Failed to send goal response 3364528e... (timeout):
                             client will not receive response
```

## 根因：每点 fork 一个 `ros2 action send_goal` CLI

```python
def send_goal(gx, gy):
    cmd = 'timeout 240 ros2 action send_goal /navigate_to_pose ...'
    return run(cmd, timeout=260)
```

三个连锁故障：

| # | 故障 | 后果 |
|---|---|---|
| ① | 每次重新建节点、重新发现 action server | 机器忙时目标根本没发出去，但脚本报「导航失败」 |
| ② | `timeout` 掐掉 CLI 后 server 侧留悬空 goal handle | `Failed to send goal response ... client will not receive response`，下一条被拖死 |
| ③ | 只能从人读文本 grep `SUCCEEDED` | 拿不到 `GoalStatus` 码，分不清「没被接受」和「接受了但超时」 |

**这是累积效应，不是西侧通道那个位置有问题。** 前面已成功跑完 3 个点，第 4 个点撞上残留客户端。

## 改法：常驻 action client

新增 `tools/verify/sim_fullchain/nav_client.py`：一个进程、一个节点、一个 `ActionClient`，
从 stdin 逐行读坐标，每行发一个目标、等真实 `result`，回一行 JSON。

```
stdin : "<x> <y>" 每行一个
stdout: {"state": "SUCCEEDED"|"ABORTED"|"CANCELED"|"TIMEOUT"|"REJECTED"|"NO_SERVER",
         "t": 秒, "detail": "..."}
```

`nav_patrol.py` 里 `send_goal()` + `parse_result()` 整块换成 `GoalClient` 类，
主循环抽成 `_run_route(route, prev, rows, client)`，整轮共用一个 client。

关键设计：**把「目标没送到」单列成 `NO_SERVER` / `REJECTED` 两个终态**，
在输出里显式标注「目标未送达」，与「送了但超时/失败」分开报。
S33 的误诊就是因为这两件事共用了 `NOT_ACCEPTED` 一个返回值。

## 验证

单点（修复前后同一个目标）：

| | 结果 | 用时 | 误差 |
|---|---|---|---|
| 修复前 | `NOT_ACCEPTED` | 240.5 s | 2.033 m |
| 修复后 | `SUCCEEDED` | 20.0 s | 0.085 m |

完整 8 点：

```
合计 8 点，PASS 8，FAIL 0
东南角    0.117m  42.5s  PASS
南走廊中点 0.111m  31.8s  PASS ~
西南角    0.090m  29.1s  PASS
西侧通道  0.085m  20.0s  PASS
西北角    0.101m  23.6s  PASS
北走廊中点 0.176m  12.3s  PASS
东北角    0.170m  11.7s  PASS
东侧通道  0.103m  22.8s  PASS
```

Nav2 侧交叉验证：`grep -c "Goal succeeded"` = **16**，
`Begin navigating` 16 条，每个目标点都真实抵达，无一条缺失。

## 教训

1. **`NOT_ACCEPTED` 是兜底返回值，不是诊断结论。** 凡是 `parse_result` 这类
   「按关键字猜」的函数，最后那个 `return` 分支必须单独命名成「未知/未送达」，
   不能和任何真实语义混用。
2. **「目标没送到」与「导航失败」是两条完全不同的排查路径。**
   前者查客户端/发现/超时，后者查控制器/规划器/代价图。混成一个词就会误诊。
3. **一次性 CLI 在重负载下不可靠。** `ros2 action send_goal`、`ros2 service call`
   这类命令每次都要重新发现，批量循环调用必然累积失败。
   要循环就该用常驻节点。
4. **日志里被忽略的 WARN 往往就是根因。** `client will not receive response`
   在失败前 4 秒就打印了，当时只盯着 `NOT_ACCEPTED`。
5. **算出来的指标会骗人。** `2.03 m / 240.5 s = 0.008 m/s` 看着像爬行，
   实际分母是超时上限。指标必须附「来源」，是实测还是推导。
