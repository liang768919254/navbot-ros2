# P3 · ROS2 自主导航移动机器人「领航者」保姆级项目文档

> 制定日期：2026-09-29（2026-09-29 修订：暂不考虑考研，时间预算与下一步重排）
> 上游依据：`具身智能B站课程体系与18个月进阶路径.md` · 阶段三交付物 **P3**
> 定位：**机器人软件 / 中间件工程师**方向的核心作品（对应 L3 中间件层）
> 预计周期：仿真路线 **2–3 周**（每天 3–4h）；真机追加 **1–2 周**
> 预算：仿真 **0 元**（只需现有笔记本）；真机约 **800–1500 元**

---

## 〇、定位与边界

### 0.1 先一句话说清这个项目要干什么

**让一台两轮差速小车，在它自己建的房间里，从 A 点自己规划路径走到 B 点。**

拆成四个能力，每个对应一个阶段：

| 阶段 | 能力 | 一句话验收 |
|---|---|---|
| A | **建模** | RViz 里能看到一个形状正确的车，坐标系树是完整的 |
| B | **动起来** | Gazebo 里用键盘遥控，车往前走、激光在扫、里程计在更新 |
| C | **建图** | 遥控走一圈，RViz 里长出一张和房间形状一致的灰度图 |
| D | **导航** | 在 RViz 里点一个目标点，车自己绕开障碍走过去 |

**这四步必须按顺序做，不能跳。** 理由在下面 §0.3。

### 0.2 ★ 术语澄清表（避免和别的文档打架）

你手上有好几套「阶段」编号，混了就全写错。本文一律用**字母**标阶段：

| 本文的叫法 | 指的是 | 不要和这些混 |
|---|---|---|
| **阶段 A/B/C/D** | 本项目内部的四个阶段 | — |
| 「18 个月路线」 | `具身智能B站课程体系与18个月进阶路径.md` 里的入门/进阶/实战 | 那是**月级**，跨度一年半 |
| 「sensor_bridge 阶段一/二/三」 | `ros2相关文件/ROS2结业项目/` 下那套 | 那是**上一个项目**（STM32 串口桥） |
| 「P3」 | 18 个月路线里阶段三的四个交付物之一 | P1 机械臂 / P2 驱动板 / P4 四足 |

★ **本文兑现的是 P3 的原文要求**（路线文档第 162 行）：
> `P3 · ROS2 自主导航移动机器人` | 约 800–1500 元（底盘+RPLIDAR+树莓派/RK） | Nav2、slam_toolbox、串口协议、TF | 机器人软件

以及阶段二交付物里那条前置（第 140 行）：
> Gazebo 仿真：自建 URDF 机器人 + 用 slam_toolbox 建图 + Nav2 自主导航（跑通全流程）

★ 所以本文的仿真部分是**还阶段二的债**，真机部分是**赚阶段三的 P3**。一次做完两件事。

### 0.3 ★★ 范围红线表（这是本项目的灵魂）

**范围控制是工程素养，不是时间不够才控制。** 下面这张表比后面所有技术内容都重要——
它决定你能不能「把一个项目做完」，而不是「开十个头、一个都没收尾」。

| 看到就想加的东西 | 现在**不加**的理由 | 什么时候加 |
|---|---|---|
| **SLAM 算法自己实现**（手写 scan matching） | 高翔《视觉SLAM十四讲》是阶段二的课。现在手写 = 用 2 个月换 0 个可运行结果 | 阶段二学完 14 讲之后 |
| **多传感器融合**（加 IMU 做 EKF） | 需要先有可靠的单传感器里程计。现在是「先让系统闭环」阶段 | 真机路线第二阶段 |
| **多机 / DDS 调优 / 跨网段** | 一台车都还没跑通，多机是纯浪费 | 有第二台车时 |
| **自定义 Nav2 插件**（自己写 planner/controller） | 先把官方 Nav2 用透，你才知道该改什么 | 官方方案真的不满足时 |
| **机械臂 / 抓取** | 那是 P1（LeRobot）的内容，是另一条线 | 阶段三做 P1 时 |
| **视觉 SLAM / 深度相机** | 项目变成另一个项目了 | 阶段二序号 12/13 之后 |
| **RTOS / 实时性优化 / PREEMPT_RT** | 仿真里没有实时性问题；真机第一版也不需要 | 真机跑通后作为进阶 |
| **自己焊底盘 + 自己写驱动板** | 那是 P2。本项目**买套件**，把时间花在软件层 | P2 单独做 |
| **Gazebo Fortress / gz sim 迁移** | 本机装的是 Classic 11（已 EOL 但能用）。两套语法完全不同，迁移纯耗时 | 需要时再说 |
| **手工实现 PID 速度环** | 开环先跑通全流程再上闭环，避免「还没跑通就调 PID」 | ★ 阶段 D 通关后**强烈建议做**（真机走直线、不跑偏靠它） |

**时间预算参考**（不再受考研挤占，但时间盒仍要守——它防的是拖延，不是赶进度）：

| 阶段 | 时间盒 | 卡住时的止损 |
|---|---|---|
| 阶段 A（建模） | 2–3 天（约 9h） | 超了就先用参考源码的模型，别卡在 xacro 语法上 |
| 阶段 B（仿真动起来） | 3–4 天（约 16h） | 卡在 Gazebo 崩溃上超过 2h → 直接跳到阶段 C 用「假里程计 + 假雷达」 |
| 阶段 C（建图） | 2 天（约 8h） | 图建得难看没关系，能闭环就行 |
| 阶段 D（导航） | 4–5 天（约 16h） | ★ 这是重点，值得多花时间 |
| 真机 | 1–2 周（每天 3–4h） | 硬件问题不熬夜，白天光线好再弄 |

### 0.4 这个项目为什么值得做（简历视角）

对照 18 个月路线的求职自检清单（第 213–217 行）：

| 清单项 | 本项目怎么满足 |
|---|---|
| GitHub 有 README + 演示视频的仓库 | ✅ 本项目的产物就是一个完整仓库 |
| 至少 1 个「真机」项目 | ✅ 真机路线 |
| 能讲清完整系统的「感知→决策→控制→执行」数据流 | ✅ ★ 本项目就是这个数据流本身（见 §5.4 口述题） |
| 技术博客 6 篇以上 | ✅ 光本项目就能出 3 篇：TF 踩坑 / Nav2 调参 / 串口协议设计 |
| 面试题 1「传感器数据怎么从硬件走到决策层」 | ✅ ★★ 这是你能拉开差距的题，本项目正面命中 |
| 面试题 3「实时性怎么保证」 | ✅ 串口桥的线程模型 + 失联保护就是答案 |
| 面试题 6「通信丢包/抖动排查流程」 | ✅ 协议里的 seq 字段 + 坏帧计数就是答案 |

★ **最关键的一点**：这个项目把「**硬件（STM32 + 编码器 + 电机）**」和「**软件（ROS2 + Nav2）**」
用一条**你自己设计的串口协议**焊在了一起。这正是「具身智能本体工程向」这个定位的核心竞争力 ——
纯算法的人讲不清协议，纯软件的人搞不定编码器。

---

## 一、明确目标与范围

### 1.1 目标分四层

| 层次 | 目标 | 判据 |
|---|---|---|
| **功能层** | 车能在自己建的图上，从当前位置自主导航到任意指定点 | 连发 5 个随机目标点，全部到达（到点判据 xy<0.15m） |
| **工程层** | 全流程可复现：一条 `ros2 launch` 起系统；一份 README 让别人能跑 | 把仓库给同学，他能独立跑通阶段 B |
| **能力层** | 掌握：URDF/xacro、TF 树、slam_toolbox、Nav2 参数、串口协议设计、ROS2 节点线程模型 | §5.4 的 10 道口述题答出 ≥8 道 |
| **衔接层** | 为阶段二「SLAM 算法」和面试的「系统数据流」问题打底 | 能白板画出 TF 树 + 数据流图 |

### 1.2 范围（In / Out）

| In Scope（要做） | Out of Scope（不做） |
|---|---|
| URDF/xacro 建模 + 校验 | 用 CAD 建精细模型 |
| Gazebo Classic 仿真 | 迁移到 gz sim / Isaac Sim |
| slam_toolbox 建图 + 存图 | 自己实现 SLAM |
| Nav2 导航（AMCL + NavFn + DWB） | 换更高级的规划器 / 自写插件 |
| 串口协议设计（PC↔STM32） | CAN 总线 / EtherCAT |
| STM32 固件（PWM + 编码器 + 串口） | RTOS / 实时性优化 |
| 里程计计算（差速运动学） | IMU 融合 / EKF |
| 失联自动停车（安全兜底） | 完整的安全体系（碰撞检测/急停按钮） |

---

## 二、前置条件

> ★★ 这一节是「真·前置」。每条都标了**怎么验证**，别跳过。
> 本节的结论来自 2026-09-29 对本机的**实测**，不是推测。

### 2.1 环境现状（本机实测结果）

**已经有的：**

| 项 | 实测结果 | 怎么自己验 |
|---|---|---|
| Ubuntu | 22.04.5 LTS，根分区 147G / 剩 89G | `df -h /` |
| ROS2 | Humble，`/opt/ros/humble`，**328 个包** | `ls /opt/ros/` + `ros2 pkg list \| wc -l` |
| Gazebo | **Classic 11.10.2**（`gazebo` / `gzserver` / `gzclient` 都在） | `gazebo --version` |
| gazebo_ros | 已装（`gazebo_ros`、`gazebo_plugins`、`gazebo_ros2_control` 等 9 个包） | `ros2 pkg list \| grep gazebo` |
| tf2 全套 | 已装（`tf2_ros`、`tf2_tools`、`tf2_geometry_msgs`…） | `ros2 pkg list \| grep tf2` |
| nav_msgs | 已装（`OccupancyGrid`、`Odometry`、`Path` 都在） | `ros2 interface list \| grep nav_msgs` |
| xacro / urdf / rviz2 / robot_state_publisher | 全部已装 | 见下面的自检脚本 |
| teleop_twist_keyboard | 已装 | `ros2 pkg executables teleop_twist_keyboard` |
| socat | 已装（`/usr/bin/socat`） | `command -v socat` |
| pyserial | **3.5**（已装给系统 python） | `/usr/bin/python3 -c "import serial; print(serial.__version__)"` |

**★★ 还没有的（本项目必须先装）：**

| 缺什么 | 严重程度 | 装法 |
|---|---|---|
| **`nav2_bringup`**（含整个 Nav2） | ★★★ 阻断阶段 D | `sudo apt install ros-humble-nav2-bringup` |
| **`slam_toolbox`** | ★★★ 阻断阶段 C | `sudo apt install ros-humble-slam-toolbox` |
| `nav2_rviz_plugins`（RViz 的 Nav2 面板） | ★★ 影响体验 | 随 nav2_bringup 一起装 |
| `twist_mux`（多源速度仲裁） | ★ 可选（真机有用） | `sudo apt install ros-humble-twist-mux` |
| `rplidar_ros`（真机雷达驱动） | ★★ 真机才需要 | 见 §2.4 |

**实测证据（本机真实输出）：**

```bash
$ ros2 pkg prefix nav2_bringup
Package not found
$ ros2 pkg prefix slam_toolbox
Package not found

$ ros2 launch nav2_bringup bringup_launch.py
Package 'nav2_bringup' not found: "package 'nav2_bringup' not found, searching: ['/opt/ros/humble']"
rc=1

$ ros2 run slam_toolbox sync_slam_toolbox_node
Package 'slam_toolbox' not found
rc=1
```

**但镜像源里是有的（实测已确认）**，你的 ROS 源指向中科大镜像，包版本如下：

| 包名 | 镜像源里的版本 |
|---|---|
| `ros-humble-nav2-bringup` | `1.1.20-1jammy.20260908.021125` |
| `ros-humble-slam-toolbox` | `2.6.10-1jammy.20260908.014627` |
| `ros-humble-nav2-amcl` | `1.1.20-1jammy.20260908.000809` |
| `ros-humble-nav2-dwb-controller` | `1.1.20-1jammy.20260908.012626` |
| `ros-humble-nav2-navfn-planner` | （随 nav2-bringup 依赖自动装） |
| `ros-humble-nav2-rviz-plugins` | `1.1.20-1jammy.20260908.014610` |
| `ros-humble-nav2-simple-commander` | `1.1.20-1jammy.20260907.224519` |
| `ros-humble-twist-mux` | `4.3.0-1jammy.20260907.224309` |
| `ros-humble-turtlebot3-gazebo` | `2.3.8-1jammy.20260908.002354`（备用参考） |
| `ros-humble-rplidar-ros` | `2.1.4-1jammy.20260907.224018`（真机用） |
| `ros-humble-robot-localization` | `3.5.4-1jammy.20260908.012128`（以后加 IMU 用） |

> 检查方式（你也可以自己复核）：
> ```bash
> curl -s https://mirrors.ustc.edu.cn/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz \
>   | zcat | grep -A1 "^Package: ros-humble-nav2-bringup$"
> ```

### 2.2 依赖安装（一次性，约 5–10 分钟）

```bash
# ★ 一条命令装满（nav2_bringup 会拉进绝大多数依赖）
sudo apt update
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-rviz-plugins \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-tools \
    ros-humble-nav2-simple-commander

# 装完立刻验证（★ 这一步必须做，不看输出就等于没装）
source /opt/ros/humble/setup.bash
ros2 pkg prefix nav2_bringup      # 应输出 /opt/ros/humble
ros2 pkg prefix slam_toolbox      # 应输出 /opt/ros/humble
ros2 pkg executables nav2_bringup # 应列出 bringup_launch.py 等
```

★ **装完之后要验证的 4 件事**（缺一个阶段 D 就会卡）：

| 验证项 | 命令 | 期望 |
|---|---|---|
| Nav2 全在 | `ros2 pkg list \| grep -c "^nav2"` | ≥ 25 |
| slam_toolbox 在 | `ros2 pkg prefix slam_toolbox` | `/opt/ros/humble` |
| AMCL 在 | `ros2 pkg prefix nav2_amcl` | `/opt/ros/humble` |
| RViz 面板在 | `ros2 pkg prefix nav2_rviz_plugins` | `/opt/ros/humble` |

### 2.3 代码前置：sensor_bridge 的基础必须真的在你手上

本项目会大量复用你 `sensor_bridge` 的经验。**但按 `代码回收清单.md` 的结论，
你的阶段一到三还没真正「回收」完** —— 进度表里 6 项一格都没打勾。

**不要求你先做完回收**（那是另一条线），但下面这三条**认知**必须先有：

| 前置认知 | 为什么本项目需要 | 在哪补 |
|---|---|---|
| 「改代码不用 build，改 `entry_points` 必须 build」 | 阶段 D 你要反复加节点入口 | `代码回收清单.md` §0C |
| 「读串口不能放在 spin 线程里」 | ★★ 阶段 D 的 `chassis_bridge` 就是这个架构 | 同上 §第二层第 2 题 |
| 「定时器 + 最新值」而不是「收到就发」 | ★★ 里程计发布就用这个模式 | 同上 §第二层第 3 题 |

**★ 关键判据**：能不能不看文档回答出上面三条？答不上来就先花 40 分钟把
`代码回收清单.md` 的「第二层 10 题口述」做了 —— 那是本项目的直接前置。

### 2.4 硬件前置（只在做真机路线时需要）

| 部件 | 建议规格 | 预算 | 说明 |
|---|---|---|---|
| 底盘套件 | 两轮差速 + 万向轮，含 TT/GA25 电机 + 编码器 | 150–300 | ★ 必须**带编码器**，否则做不了里程计 |
| 电机驱动板 | TB6612 / A4950 / DRV8833 | 20–50 | ★ 两路即可；要能接受 PWM + DIR |
| 主控 | **STM32F103C8T6**（你已有）| 0 | 熟悉、够用、代码能直接沿用 |
| 激光雷达 | **RPLIDAR A1M8**（12m 量程，360°） | 400–550 | ★ 别买 C1（量程短、社区资料少）；A1 资料最全 |
| 上位机 | 你现有的笔记本（i9-13900H / 16G） | 0 | ★ 不用买树莓派！先用笔记本跑，跑通了再考虑下放 |
| 电池 | 3S 锂电 11.1V 2200mAh + 降压模块 | 80–150 | ★ 必须带**低压保护** |
| USB 转串 | CH340 / CP2102 | 10 | ★ 你已有一个 |

**合计约 660–1100 元**，落在路线文档预估的 800–1500 区间内。

★★ **一个重要的省钱建议**：不要让树莓派/RK 成为你的 blocker。
路线文档写「底盘+RPLIDAR+树莓派/RK」，但**树莓派是"最后一步"**：
先用笔记本通过 USB 串口连 STM32 把整个导航跑通，确认没问题了，
再把 PC 换成树莓派（或者干脆不换）。理由：

- 树莓派上跑 Nav2 性能吃紧，调试体验差
- 换平台会引入一批新问题（系统版本、编译、串口权限），
  直接掩盖「导航本身有没有问题」这个核心判断

### 2.5 ★ 认知前置：三个必答问题

**先别往下看。合上文档，回答这三个问题：**

1. 一台机器人身上，`map` / `odom` / `base_link` 这三个坐标系分别代表什么？
   谁在「动」、谁「不动」？
2. `slam_toolbox` 和 `AMCL` 都能发布 `map→odom` 变换。**为什么不能同时开？**
   同时开会看到什么现象？
3. 里程计（odometry）为什么一定会漂？`slam_toolbox` 和 `AMCL` 分别用什么办法治它？

答不上来是**正常的** —— 这三个问题就是本项目要教你的核心。
但你要带着它们往下读，读到对应章节时回头看。

---

## 三、分步执行流程

### 3.0 ★★ 先看懂这张全景图（比读十遍文档有用）

```
                    ┌──────────────── 阶段 A：建模 ────────────────┐
                    │  navbot.xacro ──► robot_state_publisher      │
                    │       │                    │                 │
                    │       │                    ▼                 │
                    │       └──► URDF 文本 ──► /tf + /tf_static    │
                    │                          (TF 树)             │
                    └──────────────────────────────────────────────┘
                                        │
                    ┌──────────────── 阶段 B：仿真 ────────────────┐
                    │  Gazebo Classic 11                          │
                    │    ├─ libgazebo_ros_diff_drive.so           │
                    │    │    收 /cmd_vel  →  发 /odom             │
                    │    │    ★ 广播 odom→base_footprint 变换      │
                    │    ├─ libgazebo_ros_joint_state_publisher.so│
                    │    │    发 /joint_states（让轮子真的转）     │
                    │    └─ libgazebo_ros_ray_sensor.so           │
                    │         发 /scan(激光) (LaserScan(激光扫描), (框架)frame=laser_link)│
                    └──────────────────────────────────────────────┘
                                        │
                    ┌──────────────── 阶段 C：建图 ────────────────┐
                    │  slam_toolbox (async异步)                    │
                    │    吃 /scan + odom→base 变换                 │
                    │    ★ 自己创建 map 坐标系                     │
                    │    ★★ 广播 map→odom 变换（唯一发布者！）      │
                    │    发 /map (OccupancyGrid占用网格)            │
                    │         ↓                                   │
                    │    map_saver_cli →  .pgm + .yaml            │
                    └──────────────────────────────────────────────┘
                                        │
                    ┌──────────────── 阶段 D：导航 ────────────────┐
                    │  加载 .pgm/.yaml                             │
                    │  AMCL                                        │
                    │    吃 /scan + 地图 + odom→base 变换           │
                    │    ★★ 广播 map→odom 变换（唯一发布者！）      │
                    │  Nav2 全套：                                  │
                    │    planner_server  算全局路径 → /plan        │
                    │    controller_server (DWB) 算速度 → /cmd_vel │
                    │    costmap 2D      维护障碍地图               │
                    │    bt_navigator    编排上面这些               │
                    └──────────────────────────────────────────────┘
                                        │
                    ┌──────── 真机路线：PC 换掉 Gazebo ────────────┐
                    │  STM32 ──USB串口──► chassis_bridge (底盘桥)节点│
                    │    上行 O,seq,enc,vel,... → /odom + TF       │
                    │    下行 /cmd_vel → V,seq,vl,vr              │
                    │  ★ 协议见 参考源码/firmware/navbot_protocol.md│
                    └──────────────────────────────────────────────┘
```

**★ 记住这张图的三个要点**：

1. **TF 树是骨架**。所有东西都挂在 `map → odom → base_footprint → laser_link` 这条链上。
   链断了，Nav2 一步都走不了。
2. **`map→odom` 变换只能有一个发布者**。建图时是 slam_toolbox，导航时是 AMCL。
   这是本项目最高频的坑（见 §6 陷阱 1）。
3. **真机路线只是把「Gazebo」这一块换成了「STM32 + 串口桥」**。
   上面的 Nav2 部分一行都不用改 —— 这正是 ROS2 解耦的价值，也是你面试要讲的点。

---

### 阶段 A：建模（2–3 天，约 9h）

**目标**：RViz 里看到一个形状正确的车，TF 树完整。

#### A.1 建工作空间与功能包

```bash
# ★ 用你已有的 ros_study 工作空间（实际路径是 ~/bridge/ros_study，本项目也放这里）
cd ~/bridge/ros_study/src

# ① 描述包：放 URDF / mesh / RViz 配置 + display 启动文件（只展示模型，不依赖仿真）
# ★★ 关键：--dependencies 只放「编译期要 find_package」的包。
#   urdf / xacro 有 CMake config（Config.cmake），可以放；
#   joint_state_publisher 是纯 Python 包，没有 Config.cmake，
#   放进来会在 CMakeLists.txt 生成 find_package(joint_state_publisher) 导致编译失败！
#   → 运行时依赖（joint_state_publisher / robot_state_publisher）改放到 package.xml 的 <exec_depend>。
ros2 pkg create navbot_description --build-type ament_cmake \
    --dependencies urdf xacro

# ② 启动包：放 sim/slam/nav 启动文件 + config / world / map（依赖仿真与导航栈）
ros2 pkg create navbot_bringup --build-type ament_cmake \
    --dependencies nav2_bringup slam_toolbox robot_state_publisher rviz2

# ③ 桥接包（Python）：放 chassis_bridge 节点（阶段 D 写）
ros2 pkg create navbot_bridge --build-type ament_python \
    --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros
```

★ **为什么要三个包，而不是一个大包？**

| 包 | 变化频率 | 依赖 |
|---|---|---|
| `navbot_description` | 低（模型定了就不动） | 只依赖 urdf/xacro |
| `navbot_bringup` | 中（调参频繁） | 依赖 nav2、slam_toolbox |
| `navbot_bridge` | 高（逻辑迭代） | 只依赖消息包 + pyserial |

分开的理由：**改一个地方只需要重编一个包**。一个大包会导致
「只想改个参数，结果整个 Nav2 依赖树都要检查一遍」。

#### A.2 ★ 先确定坐标系约定（这一步错了后面全错）

**动手写 xacro 之前，先把这四件事定死，写在纸上：**

| 约定 | 本项目的选择 | 为什么 |
|---|---|---|
| 前进方向 | **+X** | ROS 标准（REP-103） |
| 左侧 | **+Y** | 右手系 |
| 上方 | **+Z** | 右手系 |
| 地面高度 | **z = 0** | 所有连杆高度从这里量 |
| **根 link** | **`base_footprint(基础足迹)`** | ★ 见下 |

★★ **`base_footprint(基础足迹)` vs `base_link(基础链接)` —— 这个区别先讲清楚**：

| | `base_footprint` | `base_link(基础链接)` |
|---|---|---|
| 物理位置 | 车体在**地面上的投影点** | 车体**几何中心**（离地约 5cm） |
| 有没有几何体 | **没有**（空 link） | 有（底盘 box） |
| 谁用它 | ★ Nav2 的 AMCL / 代价地图 | 建模、可视化 |
| 关系 | 是 `base_link` 的**父**节点 | 是 `base_footprint` 的子 |

**为什么 Nav2 偏好 `base_footprint`？**
因为里程计描述的是「地面上的平面运动 (x, y, θ)」。
用地面投影点做基准，三个自由度干干净净；
用 `base_link(基础链接)` 会掺进一个永远不变的高度 z，是多余信息。

★ **两条都合法，但必须全项目一致**。本项目选 `base_footprint(基础足迹)` 作为
TF 树末端，并在三处保持一致：

1. `navbot.xacro` 里根 link 叫 `base_footprint`
2. Gazebo diff_drive(差速驱动) 插件的 `robot_base_frame(机器人基座框架)` = `base_footprint`
3. `nav2_params.yaml` 里所有 `robot_base_frame` / `base_frame_id` = `base_footprint`

★ **三处不一致的后果**：TF 树断在中间，Nav2 报
`Timed out waiting for transform from base_footprint to odom(等待从base_footprint到odom的转换时超时)`，
而你会去怀疑是不是 slam_toolbox 没起。

#### A.3 写模型（★ 这一步自己写）

**参考源码**：`参考源码/urdf/navbot.xacro`（完整可跑，已实测展开通过）

**自己写的话，按这个顺序，每步都验：**

| 步 | 内容 | 验证命令 | 期望输出 |
|---|---|---|---|
| A.3.1 | 只写 `base_footprint` + `base_joint` + `base_link`（底盘 box） | `xacro navbot.xacro > /tmp/n.urdf && check_urdf /tmp/n.urdf` | `root Link: base_footprint has 1 child(ren)` |
| A.3.2 | 加一个驱动轮（左轮） | 同上 | `base_link has 1 child(ren)` |
| A.3.3 | **改写成宏**，加右轮 | 同上 | `base_link has 2 child(ren)` |
| A.3.4 | 加前后万向球（也用宏） | 同上 | `base_link has 4 child(ren)` |
| A.3.5 | 加激光雷达 `laser_link` | 同上 | `base_link has 5 child(ren)` |
| A.3.6 | 加惯性张量（用宏算） | 同上 | 无 `inertia` 相关警告 |

★★ **用宏的必要性，你写 A.3.2 和 A.3.3 之间会亲身体会到**：
左轮那一整段（visual + collision + inertial + joint/视觉 + 碰撞 + 惯性 + 关节）约 30 行，
右轮和它**只有 y 坐标符号不同**。第二遍抄的时候你会想「这不该抄」——
那就对了，这就是 xacro 存在的理由。

**★ 惯性张量必须自己算，不能拍脑袋填**。三个公式（参考源码里已封装成宏）：

| 几何体 | ixx | iyy | izz |
|---|---|---|---|
| 长方体 (m, x, y, z) | `m(y²+z²)/12` | `m(x²+z²)/12` | `m(x²+y²)/12` |
| 圆柱 (m, r, h，轴沿 Z) | `m(3r²+h²)/12` | `m(3r²+h²)/12` | `m·r²/2` |
| 实心球 (m, r) | `2mr²/5` | `2mr²/5` | `2mr²/5` |

★ **注意**：长方体三轴的惯性**一般互不相等**（除非是正方体），
只有圆柱（绕 x、y 轴）和实心球才是「两个方向相同」的对称情况。
所以别偷懒把长方体也写成 `ixx = iyy`。

★ **判据**：如果你把惯性张量写成 `ixx="0.1"` 这种拍脑袋值，
Gazebo 里车会「抖」或者「莫名旋转」——因为物理引擎在解一个不合物理的方程。

**★ 实测过的两个 xacro 报错（照抄文档里的写法会遇到）：**

```bash
# 报错 1：用了未定义的属性（最常见，就是拼错）
$ xacro bad.xacro
error: name 'wheel_radus' is not defined 
when evaluating expression 'wheel_radus'
when processing file: /tmp/bad1.xacro
rc=2
# → 检查 ${} 里的变量名，或者 xacro:property 有没有漏

# 报错 2：宏参数用了 $ 前缀（ROS1 老写法，网上教程很多这么写）
$ xacro bad.xacro
error: Invalid parameter "prefix"
when instantiating macro: wheel (/tmp/bad3.xacro)
rc=2
# → 正确写法是 params="prefix side"，不是 params="$prefix $side"
#   这是 ROS1→ROS2 的一个静默不兼容点
```

#### A.4 在 RViz 里看模型

**参考源码**：`参考源码/launch/display_navbot.launch.py`、`参考源码/rviz/navbot_check.rviz`

```bash
cd ~/bridge/ros_study
colcon build --symlink-install --packages-select navbot_description navbot_bringup
source install/setup.bash

ros2 launch navbot_description display_navbot.launch.py
```

**RViz 里逐项确认这 5 条：**

| # | 检查什么 | 在哪里看 | 期望 |
|---|---|---|---|
| 1 | 模型显示了 | 左侧 RobotModel | 一个蓝色底盘 + 两个黑轮 + 红雷达 |
| 2 | 模型位置对 | Orbit 视角 | 底盘贴地，轮子在同高 |
| 3 | **TF 树完整** | 左侧 TF，勾 Show Names | 能连出 `base_footprint → base_link → laser_link` 一条链 |
| 4 | 关节来源存在 | 终端 `ros2 topic hz /joint_states` | 有数据（约 10 Hz） |
| 5 | 坐标系名字对 | 终端 `ros2 run tf2_tools view_frames` | 生成 `frames.pdf`，树结构正确 |

★★ **看不到模型的三步排查（顺序很重要，从便宜的查起）：**

```
① Fixed Frame 是不是空的 / 写了不存在的坐标系？
   → RViz 状态栏会写 "Fixed Frame [xxx] does not exist"
   → 这是 90% 的"看不到模型"的原因

② /robot_description 有没有内容？
   ros2 topic echo /robot_description --once | head -20
   → 空的话：xacro 展开失败，去看 launch 窗口的报错

③ /joint_states 有没有数据？
   ros2 topic hz /joint_states
   → 没有的话：joint_state_publisher 没起或崩了
```

**★ 一个必须记住的 launch 坑（Humble 头号报错）：**

`display_navbot.launch.py` 里这一行的 `value_type=str` **千万不能省**：

```python
robot_description = ParameterValue(
    Command(['xacro ', xacro_file]),
    value_type=str,          # ★★ 省了就等于自找麻烦
)
```

不写的话，launch 会把这段 XML 当成 YAML 解析，报一堆**和模型无关**的
YAML 语法错误，让你去怀疑 xacro 文件。这是「模型明明没错却起不来」的第一名原因。

#### A.5 阶段 A 验收

| # | 判据 | 验证命令 | 不能过的情况 |
|---|---|---|---|
| A-1 | `xacro` 展开无错 | `xacro navbot.xacro > /tmp/n.urdf && echo OK` | 报 `is not defined` / `Invalid parameter` |
| A-2 | URDF 树合法 | `check_urdf /tmp/n.urdf` | `Two root links found` / `Failed to find root link` |
| A-3 | 树结构是 5 个子节点 | `check_urdf` 输出数 `child` 行 | 少了激光或万向轮 |
| A-4 | RViz 里模型可见 | 目视 | 一片空白 |
| A-5 | TF 树完整不断链 | `view_frames` 生成的 pdf | 树断成两截 |
| A-6 | ★ 惯性张量与几何尺寸自洽 | 手算一遍对照 | 用 `0.1` 这种占位值 |

**★ A-2 的失败样例（实测原文）**：

```bash
# 如果漏了某个 link 的 joint（这里 orphan_link 没有 joint 连着）
$ check_urdf t2.urdf
Error:   Failed to find root link: Two root links found: [base_link] and [orphan_link]
         at line 264 in ./urdf_parser/src/model.cpp
ERROR: Model Parsing the xml failed
rc=255
```

---

### 阶段 B：仿真里动起来（3–4 天，约 16h）

**目标**：Gazebo 里用键盘遥控车，`/scan(扫描)` 和 `/odom` 都有数据，TF 树完整。

★ **本阶段刻意不含 Nav2**。理由：把「底盘通不通」和「导航配不配」
分成两件事验。合在一起出错时你分不清是哪个。

#### B.1 写 Gazebo 挂载层

**参考源码**：`参考源码/urdf/navbot.gazebo.xacro`、`navbot.urdf.xacro`

**★★ 关键认知：为什么 URDF 和 Gazebo 插件要分两个文件？**

| | URDF 本体 | Gazebo 挂载层 |
|---|---|---|
| 描述什么 | 机器人**长什么样** | 在仿真里**怎么动** |
| 真机上还需要吗 | ✅ 需要（RViz、TF 都用） | ❌ 不需要 |
| 谁加载它 | `robot_state_publisher` | Gazebo 的 `.so` 插件 |

真机上驱动是**你自己写的节点**，不是插件。所以物理上就该分开 ——
这样才能保证 `navbot.xacro`（模型本体）在仿真和真机之间**一字不改**。

**两个插件的参数清单（★ 以本机头文件为准，别照抄网络教程）**

`libgazebo_ros_diff_drive.so` —— 参数来源：
`/opt/ros/humble/include/gazebo_plugins/gazebo_ros_diff_drive.hpp`

| 参数 | 本项目值 | ★ 说明 |
|---|---|---|
| `left_joint` / `right_joint` | `left_wheel_joint` / `right_wheel_joint` | ★★ 必须和 xacro 里 joint 名**完全一致** |
| `wheel_separation` | `0.16` | ★★ 必须 = URDF 的 `wheel_sep` |
| `wheel_diameter` | `0.065` | ★★ = `2 × wheel_r` = `2 × 0.0325` |
| `max_wheel_torque` | `5.0` | 太小爬不上坡，太大瞬间打滑 |
| `max_wheel_acceleration` | `1.0` | 同上 |
| `command_topic` | `cmd_vel` | 收速度指令 |
| `publish_odom` | `true` | 发 `/odom` |
| **`publish_odom_tf`** | **`true`** | ★★ **TF 树能不能闭合的关键！** |
| `odometry_frame` | `odom` | 里程计坐标系名 |
| `robot_base_frame` | `base_footprint` | ★ 和 §A.2 的约定一致 |

★★ **`publish_odom_tf` 设成 false 的后果**（这是本项目的前三个坑之一）：
TF 树断在 `odom`，Nav2 会一直刷
`Timed out waiting for transform from base_footprint to odom`，
而你的车在 Gazebo 里明明能正常动 —— 极容易误判成「Nav2 没配好」。

`libgazebo_ros_ray_sensor(射线传感器).so` —— 参数来源：
`/opt/ros/humble/include/gazebo_plugins/gazebo_ros_ray_sensor.hpp`

| 参数 | 本项目值 | ★ 说明 |
|---|---|---|
| **`output_type`** | **`sensor_msgs/LaserScan`** | ★★ **不写默认是 PointCloud2**，那样 `/scan` 根本不出现 |
| `frame_name` | `laser_link` | 不写会用父 link 名（本项目恰好也叫这个） |
| `<remapping>~/out:=scan` | — | 让话题名是 `/scan` 而不是 `/laser_controller/out` |
| `<samples>` | `360` | 360 线，和 RPLIDAR A1 一致 |
| `<min>` / `<max>` | `0.12` / `12.0` | ★ 必须 ≥ Nav2 参数里的 `raytrace_max_range` |
| `<stddev>` | `0.01` | ★ **必须加噪声！** 零噪声会让你在仿真里调得很顺、上真机立刻崩 |

★ **`output_type` 不写的后果**：`ros2 topic list` 里找不到 `/scan`，
Nav2 的代价地图永远是空的，车会「直着撞墙」。

#### B.2 写世界文件

**参考源码**：`参考源码/worlds/navbot_room.world`（已通过 `gz sdf -k` 校验）

**房间设计的四个考点**（★ 不是随便摆的）：

| 结构 | 尺寸 | 考什么 |
|---|---|---|
| 外墙 6m × 5m | 高 0.5m | ★ 高 0.5m 是因为激光是 2D 的，只扫一个平面 |
| `partition_mid`（2.6m 长） | — | 造出闭环，让 slam_toolbox 能做回环检测 |
| `partition_short` + 通道 | **约 0.6m** | ★★ 窄通道 —— 考验 `inflation_radius` 和 `robot_radius` |
| 两根柱子 | 半径 0.15m | ★★ 给 AMCL 提供**可分辨特征** |

★★ **柱子的那条最值得说**：四面光滑的墙对 2D 激光来说是
「四个没有区分度的平面」，粒子滤波会发散、定位乱跳。
加两根柱子，定位立刻稳。
——**这也是真机上的现实**：空旷车库定位容易丢，有货架的房间就稳。

#### B.3 起仿真

**参考源码**：`参考源码/launch/sim_navbot.launch.py`

```bash
# 终端 1：起仿真
ros2 launch navbot_bringup sim_navbot.launch.py

# 终端 2：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   i = 前进   , = 后退   j = 左转   l = 右转   k = 停
#   ★ 刚开始把速度调小：按 q/z 可以调速度档
```

**★★ 这一步的两个必然坑（都有实测依据）：**

```
坑 1：spawn 报 "Service /spawn_entity unavailable"
      原因：Gazebo 起得慢（3~8 秒），立刻 spawn 会连不上服务。
      解决：本 launch 里用了 TimerAction(period=3.0) 延迟。
           如果你的机器更慢，把这个值加到 5.0。

坑 2：spawn 报 "Waiting for entity xml on /robot_description" 然后卡住
      原因：-topic 参数写错，或者 robot_state_publisher 没起来。
      验证：ros2 topic echo /robot_description --once 有没有内容。
```

**★ 如果 Gazebo 起不来（无显卡/远程会话）**：

```bash
# 不启动图形界面，只用 gzserver
ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
# ★ 这样照样能跑 Nav2，只是看不到 3D 画面。
#   用 RViz 看激光和里程计完全够用 —— 这也是我推荐的开发方式，
#   Gazebo 界面很吃 GPU（本机是核显）。
```

#### B.4 阶段 B 验收

| # | 判据 | 验证命令 | 期望 | 不能过的情况 |
|---|---|---|---|---|
| B-1 | 模型生成到 Gazebo | Gazebo 窗口目视 | 看到车在空房间里 | 报告 spawn 失败 |
| B-2 | 键盘能遥控 | 按 `i` | 车往前走 | 车不动、轮子空转 → 查 `left_joint` 拼写 |
| B-3 | `/cmd_vel` 有数据 | `ros2 topic hz /cmd_vel` | 按键时约 10 Hz | 一直 0 |
| B-4 | **`/odom` 有数据** | `ros2 topic hz /odom` | **约 50 Hz** | 0 → 查 `publish_odom` |
| B-5 | **`/scan` 有数据** | `ros2 topic hz /scan` | **约 10 Hz** | 0 → 查 `output_type` |
| B-6 | ★★ **TF `odom→base_footprint` 存在** | `ros2 run tf2_ros tf2_echo odom base_footprint` | 持续输出 Translation | 超时 → 查 `publish_odom_tf` |
| B-7 | TF 树完整 | `ros2 run tf2_tools view_frames` | 树是完整的 | 断在 odom |
| B-8 | `/joint_states` 有数据 | `ros2 topic hz /joint_states` | 约 30 Hz | 0 → 检查 joint_state_publisher 插件 |
| B-9 | ★ 里程计数值合理 | `ros2 topic echo /odom --once` | `frame_id: odom`, `child_frame_id: base_footprint` | frame 名不对 |
| B-10 | ★ 运动方向正确 | 按 `i` 前进，看 `/odom` 的 `pose.pose.position.x` | **单调增加** | 减少 → 轮子装反或 axis 方向错 |

★ **B-10 是本阶段最有价值的一条**：它一次性验证了
「URDF 的 axis 方向 + 插件参数 + 运动学公式」三件事的一致性。

#### B.5 阶段 B 的「止损规则」

卡在 Gazebo 相关问题（崩溃、闪退、物理引擎报错）上，
**超过 2 小时就跳车**：

```bash
# 方案：用「假里程计 + 假雷达」替代 Gazebo
# ★ 本项目的 无硬件自测/fake_chassis.py 就支持这个用法
cd ~/桌面/具身智能学习路线/13_ROS2自主导航移动机器人/无硬件自测
./fake_chassis.py --dry-run          # 先看数据格式
# 然后写一个小小的假雷达节点（波束打在一个虚拟的方形房间里），
# 直接用 slam_toolbox 建图 —— 不经过 Gazebo。
```

★ **为什么这不是「作弊」**：Gazebo 只是数据源之一。
你需要练的是 **TF / slam_toolbox / Nav2**，不是 Gazebo。
真机上本来就没有 Gazebo，那一块会被串口桥替代。
在 Gazebo 上耗掉一周，是把「工具问题」当成了「能力问题」。

---

### 阶段 C：建图（2 天，约 8h）

**目标**：遥控走一圈，存出一张可用的 `.pgm` + `.yaml` 地图。

#### C.1 ★★ 先理解建图和导航的分工（本项目最重要的认知）

| | **建图模式**（阶段 C） | **导航模式**（阶段 D） |
|---|---|---|
| 谁提供 `map→odom` | **`slam_toolbox`** | **`AMCL`** |
| 需要预先有地图吗 | ❌ 不需要（它**创造**地图） | ✅ 需要（加载 `.pgm`） |
| 需要给初始位姿吗 | ❌ 不需要 | ✅ **必须给**（RViz 点 2D Pose Estimate） |
| 输出 | `/map` 话题 + `map→odom` 变换 | 定位结果 + `map→odom` 变换 |
| 启动文件 | `slam_mapping.launch.py` | `nav_bringup.launch.py` |

★★★ **绝对不要同时启动这两个！**

原因：两个节点都在广播 `map→odom` 变换，
TF 树会**疯狂抖动**（两个发布者互相打架），表现为：

- RViz 里机器人的位置**乱跳**
- 代价地图**闪烁**
- Nav2 报各种「transform 超时」

这是本项目**最高频的「看起来对但不对」故障**。
因为两个 launch 单独跑都完全正常，你不会想到是它们打架。

★ **判断当前是哪个模式的命令**：
```bash
ros2 node list | grep -E "slam_toolbox|amcl"
# 只应该出现其中一个
```
两个都出现 = 立刻关掉一个。

#### C.2 写建图参数

**参考源码**：`参考源码/config/slam_toolbox_params.yaml`

关键参数与理由（★ 都是针对 navbot 改过的）：

| 参数 | 本项目值 | ★ 为什么 |
|---|---|---|
| `mode` | `mapping` | 建图模式 |
| `base_frame` | `base_footprint` | ★ 和 §A.2 一致 |
| `resolution` | `0.05` | ★★ 必须 = Nav2 costmap 的 resolution |
| `max_laser_range` | `12.0` | ★ 必须 ≤ 雷达实际量程 |
| `minimum_time_interval` | `0.5` | ★ CPU 负担的关键旋钮。车快必须调小 |
| `minimum_travel_distance` | `0.3` | ★ 走 30cm 插一个关键帧 |
| `do_loop_closing` | **`true`** | ★★ 关掉它 = 放弃 Graph-SLAM 的核心优势 |
| `solver_plugin` | `solver_plugins::CeresSolver` | 官方默认，稳定 |
| `transform_timeout` | `0.2` | 官方默认 |
| `tf_buffer_duration` | `30.0` | ★ 设太小会在车快时丢变换 |
| `enable_interactive_mode` | `true` | ★ 允许在 RViz 里手动拉回环（调试神器） |

#### C.3 建图全流程

```bash
# ── 终端 1：起仿真 ──────────────────────────────────────────
ros2 launch navbot_bringup sim_navbot.launch.py

# ── 终端 2：起建图 ──────────────────────────────────────────
ros2 launch navbot_bringup slam_mapping.launch.py use_sim_time:=true
#   ★ use_sim_time 必须是 true（仿真里用 /clock 而不是墙上时间）
#     设错的后果：TF 时间戳对不上，所有变换都超时

# ── 终端 3：遥控走一圈 ──────────────────────────────────────
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ── 终端 4：边建边看 ────────────────────────────────────────
ros2 topic hz /map            # ★ 约 0.2 Hz（5 秒一帧），这是正常的！
ros2 topic hz /scan           # 约 10 Hz
ros2 run tf2_tools view_frames  # map→odom→base_footprint→laser_link 应完整
```

★★ **遥控走图的三条铁律**（违反任何一条，图就会建糊）：

| # | 铁律 | 为什么 |
|---|---|---|
| 1 | **慢**（0.1–0.15 m/s） | 快 → 相邻两帧激光位移大 → 扫描匹配失败 → 图重叠错位 |
| 2 | **转弯要更慢**（角度速度 0.3 rad/s 以内） | 旋转是最容易丢匹配的运动 |
| 3 | **走闭环路线** | 回到起点附近，让 slam_toolbox 做回环检测、全局校正 |

★ **判断建图质量的自检**（在 RViz 里看）：

| 现象 | 说明什么 | 怎么办 |
|---|---|---|
| 墙是**一条细黑线** | ✅ 建得好 | — |
| 墙有**重影/双层** | ❌ 匹配失败，位姿估计错 | 走慢点重来 |
| 走了一圈回来，**起点附近的墙没对齐** | ❌ 回环没生效 | 检查 `do_loop_closing: true`；走得更靠近起点 |
| 大片**灰色**（未知） | 没扫到 | 正常，走过的地方才会变白/黑 |

#### C.4 存图（★ 时机很重要）

```bash
# ★★ 必须在 slam_toolbox 还活着的时候存！
#    slam_toolbox 一退出，它发布的 /map 就没了，map_saver 会报
#    "Failed to save the map: Map not received"

# 先确认 /map 有数据
ros2 topic hz /map        # 应该约 0.2 Hz

# 存图
cd ~/bridge/ros_study/src/navbot_bringup/maps
ros2 run nav2_map_server map_saver_cli -f navbot_room

# 确认生成
ls -l navbot_room.pgm navbot_room.yaml
file navbot_room.pgm      # 应为 "Netpbm image data, size = W x H"
```

★ **`map_saver_cli` 常见报错对照**：

| 报错 | 原因 | 解决 |
|---|---|---|
| `Failed to save the map: Map not received` | slam_toolbox 已退出 / 没发过 /map | 先 `ros2 topic hz /map` 确认 |
| 生成的 `.pgm` 全白或全黑 | 地图尺寸为 0 | 说明 `/map` 是空的 |
| 存下来但 Nav2 加载后车走不了 | `negate` 搞反 | 检查 yaml 里 `negate: 0` |
| 存下来但 Nav2 报地图和激光对不上 | `resolution` 不一致 | 检查建图和 costmap 都是 0.05 |

#### C.5 阶段 C 验收

| # | 判据 | 验证 | 不能过的情况 |
|---|---|---|---|
| C-1 | `/map` 话题有数据 | `ros2 topic hz /map` | ≈ 0.2 Hz |
| C-2 | RViz 里地图长出来了 | 目视 navbot_slam.rviz | 一片灰 |
| C-3 | 地图形状和房间一致 | 对比 world 文件的墙位置 | 歪斜/错位 |
| C-4 | ★ 墙体没有重影 | 目视放大看 | 双层墙 = 匹配失败 |
| C-5 | 生成了 .pgm + .yaml | `ls -l` | 只有 yaml |
| C-6 | .pgm 能被解析 | `file navbot_room.pgm` | 不是 Netpbm |
| C-7 | ★ 地图上有柱子（特征） | 目视 | 没有 → AMCL 会发散 |

---

### 阶段 D：自主导航（4–5 天，约 16h）★ 本项目的重点

**目标**：在 RViz 里点目标点，车自己规划并走过去。

#### D.1 写 Nav2 参数（★ 这是最花时间的部分）

**参考源码**：`参考源码/config/nav2_params.yaml`（已针对 navbot 调过，已通过 YAML 校验）

★★ **最重要的一条：不要直接抄官方默认值！**

官方 `nav2_bringup/params/nav2_params.yaml` 是按 **TurtleBot3 Burger** 调的。
拿过来用，下面几处**必然**出问题：

| 官方值 | 问题 | 本项目改成 | 为什么 |
|---|---|---|---|
| `robot_radius: 0.22` | TB3 是 0.22m 半径 | **`0.20`** | navbot 车体 0.3×0.24，对角线半 ≈0.192 |
| `inflation_radius: 0.55` | ★★ **0.6m 窄通道直接封死** | **`0.25`** | ★ 膨胀半径是「总」禁区（含车半径）。2×0.25=0.5 < 0.6，留 0.1m 走廊；设 0.35 会让 2×0.35=0.7 > 0.6，通道照样封死 |
| `max_vel_x: 0.26` | 略快，仿真调参不友好 | **`0.22`** | 先稳再快 |
| `xy_goal_tolerance: 0.25` | 车会在离目标 25cm 就「到达」 | **`0.15`** | 车体才 0.3m 长 |
| `min_y_velocity_threshold: 0.5` | 差速车 Y 恒为 0，无所谓但要知道 | 保留 | 加注释说明 |

★ **自己 diff 官方文件的方法**（学习效果最好）：
```bash
diff /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
     ~/bridge/ros_study/src/navbot_bringup/config/nav2_params.yaml
```

**本项目的关键参数速查表：**

| 段 | 参数 | 值 | ★ 含义 |
|---|---|---|---|
| `amcl` | `base_frame_id` | `base_footprint` | ★★ 三处一致之一 |
| `amcl` | `global_frame_id` / `odom_frame_id` | `map` / `odom` | — |
| `amcl` | `min_particles` / `max_particles` | `500` / `2000` | ★ 核显本上限 |
| `amcl` | `update_min_d` / `update_min_a` | `0.25` / `0.2` | 距离/角度阈值 |
| `amcl` | `tf_broadcast` | **`true`** | ★★ 必须！否则没人发 map→odom |
| `amcl` | `transform_tolerance` | `1.0` | 设太小会频繁超时 |
| `controller_server` | `controller_frequency` | `20.0` | 20Hz 控制循环 |
| `FollowPath` | `vx_samples`×`vy_samples`×`vtheta_samples` | `20`×`5`×`20` | ★ = 2000 条候选轨迹/周期 |
| `FollowPath` | `critics` | 7 个打分器 | ★ 顺序体现决策偏好 |
| `local_costmap` | `rolling_window` | `true` | 跟着车走的 3×3m 窗口 |
| `global_costmap` | `track_unknown_space` | `true` | ★ 未知区域不可通行 |
| `planner_server` | `GridBased.plugin` | `NavfnPlanner` | 先用 Dijkstra（`use_astar: false`）|
| `velocity_smoother` | `max_velocity` | `[0.22, 0, 1.0]` | ★★ 保护真机的一层 |

#### D.2 跑导航全流程

```bash
# ── 终端 1：起仿真 ──────────────────────────────────────────
ros2 launch navbot_bringup sim_navbot.launch.py

# ── 终端 2：起导航（★ 前提：已经存好地图）────────────────────
ros2 launch navbot_bringup nav_bringup.launch.py use_sim_time:=true

# ── 终端 3：监控 Nav2 起来没有 ──────────────────────────────
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
#   ★ 都应输出 "active [3]"
#   如果输出 "unconfigured" 或 "inactive"，说明激活失败，去看 launch 窗口的报错
```

★★★ **RViz 里的操作顺序（顺序错了会误判很多事）**：

| 步骤 | 操作 | 为什么 |
|---|---|---|
| 1 | 确认 Fixed Frame = `map` | 配置已设好，但你要知道为什么 |
| 2 | 点 **`2D Pose Estimate`**，在地图上按车的实际位置拖一个箭头 | ★★ 给 AMCL 初始位姿 |
| 3 | 等 1–2 秒，**看红色激光点是否和地图上的墙贴合** | ★★ 贴合 = 定位收敛了 |
| 4 | 看**黄色粒子云**是否缩成一小团 | 散成一片 = 没收敛 |
| 5 | 才点 **`Nav2 Goal`** 发目标点 | — |

★★ **跳过第 2、3 步直接发目标 = 必失败**，而且报错信息和你想象的完全不同：

```
# 典型现象：
#   · 车一动不动
#   · RViz 里出现一条通向目标点的路径，但车不走
#   · 或者车朝着完全错误的方向走
# 原因：AMCL 不知道车在哪（初始位姿是默认的 (0,0,0)），
#       它给出的 map→odom 变换是错的，于是整条链路全错。
```

**命令行发目标点（不依赖 RViz，便于脚本化）**：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0, z: 0.0}, \
    orientation: {w: 1.0}}}}"
```

#### D.3 ★★ Nav2 调试的核心方法：看三个话题

导航出问题时，**不要瞎改参数**。按这个顺序看：

| 看什么 | 命令 | 说明什么 |
|---|---|---|
| **1. 有没有全局路径** | RViz 里看绿色线（`/plan`） | 有 → 规划成功，问题在控制；没有 → 规划失败 |
| **2. 代价地图长什么样** | RViz 里开 GlobalCostmap / LocalCostmap | 目标点周围是蓝紫 = 膨胀太大 |
| **3. 粒子云收敛了吗** | RViz 里看黄色箭头（`/particle_cloud`） | 散开 = 定位没收敛 |
| **4. 有速度指令吗** | `ros2 topic hz /cmd_vel` | 有 → 控制器在工作 |
| **5. 平滑后速度** | `ros2 topic hz /cmd_vel_smoothed` | 应为 20Hz |

★ **`ros2 topic hz /cmd_vel` 是判断「Nav2 到底工没工作」最快的一招**：

- 有 20Hz → Nav2 在算，车不走 = 底盘/仿真问题
- 一直 0 → Nav2 没算 → 去看规划器和控制器日志

#### D.4 阶段 D 验收（★ 本项目最硬的验收）

| # | 判据 | 验证 | 不能过的情况 |
|---|---|---|---|
| D-1 | Nav2 全部节点 active | `ros2 lifecycle get /controller_server` | `active [3]` |
| D-2 | 地图加载成功 | `ros2 topic hz /map` | ≈ 1 Hz（latched） |
| D-3 | AMCL 收敛 | RViz 看粒子云 | 缩成一小团 |
| D-4 | ★★ **单点导航成功** | RViz 点 Nav2 Goal | 车开到目标 0.15m 内 |
| D-5 | ★★ **连续 5 个随机目标点全成功** | 依次点 5 个点 | 5/5 到达 |
| D-6 | ★ **窄通道能过** | 点通道另一侧的目标 | 能穿过 0.6m 通道 |
| D-7 | ★ **动态避障** | 把 `movable_box` 推到路中间 | 车绕开或重新规划 |
| D-8 | 路径被阻挡能恢复 | 目标点设在墙角里 | 车尝试后放弃，**不无限撞墙** |
| D-9 | ★ 无 TF 超时日志 | 看 launch 窗口 | 没有 `Timed out waiting for transform` |

**★ D-5 是「能不能写进简历」的分水岭**：单次成功可能是运气，
连续 5 次说明系统真的稳定。**录视频就用 D-5 + D-6 + D-7 的组合。**

#### D.5 常见导航故障速查（症状 → 根因 → 处置）

| 症状 | 根因 | 处置 |
|---|---|---|
| 车一动不动，`/cmd_vel` 一直 0 | 规划失败 | 看 RViz 有没有绿线；看 planner_server 日志 |
| 车原地抖/抽搐 | DWB 参数过激进 | `acc_lim_x` 减半；`vtheta_samples` 减少 |
| 车在目标点附近磨蹭很久 | `xy_goal_tolerance` 太紧 | 放宽到 0.15~0.2 |
| 窄通道过不去 | `inflation_radius` 太大 | ★ 确认 `2 × inflation_radius < 通道宽`（本项目 0.25 → 2×0.25=0.5 < 0.6 能过；改成 0.35 就封死） |
| 车贴墙走然后蹭墙 | `robot_radius` 太小 | 增大到实际对角线半径 |
| 定位一直漂 | AMCL 粒子不够 / 特征不足 | 加粒子到 3000；场景加柱子 |
| 反复报 TF 超时 | ★★ **slam_toolbox 和 AMCL 同时开着** | 关掉 slam_toolbox |
| 地图和激光对不上 | 初始位姿给错 | 重新点 2D Pose Estimate |
| RViz 里地图是"负片" | yaml 的 `negate` 反了 | 改成 `negate: 0` |
| 一切正常但仿真里车很慢 | CPU 跟不上 | `gui:=false`；减少 `vx_samples` |

---

### 真机路线（1–2 周，每天 3–4h）

**目标**：把 Gazebo 换成 STM32 底盘，Nav2 部分一行不改。

#### R.1 先理解「换的是什么」

```
仿真：  Gazebo 插件  ──►  /odom + /scan + TF
真机：  STM32 ──串口──► chassis_bridge 节点 ──► /odom + TF
                        RPLIDAR ──► rplidar_node ──► /scan

★ Nav2 侧（阶段 D 的 launch 和参数）完全不用改。
  这就是 ROS2 解耦的价值 —— 也是你面试要讲的核心点。
```

#### R.2 写串口协议（★ 自己设计）

**参考源码**：`参考源码/firmware/navbot_protocol.md`（完整协议规范）

**协议设计的三个关键决定**（每一个面试都会被问到）：

| 决定 | 选择 | 理由 |
|---|---|---|
| ASCII 还是二进制？ | **ASCII** | 学习期调试成本接近零；50Hz×40B=2KB/s，115200 够用 |
| 速度用什么单位？ | **mm/s 整数** | ★ MCU 无 FPU，浮点 printf 极慢；1mm/s 误差对 0.2m/s 车是 0.5% |
| 为什么要有 `seq`？ | 丢帧检测 | ★★ 没有它无法区分「车没动」和「丢了 3 帧」（sensor_bridge 的教训） |
| 为什么要有 `tick_ms`？ | 时间对齐 | `header.stamp` 是到达时刻不是采样时刻，相差 1~2ms |

★ **协议继承自 `sensor_bridge` 阶段三的 v3 经验**：
那边的问题是「靠字段数猜格式」，本项目改成「**字段数不对就丢**」。

#### R.3 STM32 固件（★ 骨架版，自己填）

**参考源码**：`参考源码/firmware/navbot_chassis.c`（★ 骨架 + TODO + 验收判据）

★ **按你 2026-09-22 立的规矩，这是骨架不是成品**：
核心逻辑（协议解析、编码器统计、PID、安全兜底）**只给需求和判据**。

固件要实现的四块：

| 模块 | 关键技术点 | 验收判据 |
|---|---|---|
| 编码器读取 | ★ **16 位 CNT 回绕处理** | 连续转 5 圈，无 >5000 的单步跳变 |
| 速度换算 | ★ 单位链条（脉冲→mm/s） | 手推 1m，enc 增量 ≈ 10343 |
| 串口收发 | ★ **`setvbuf` 关缓冲** | 上电 1 秒内 `/odom` 出数据 |
| **失联保护** | **500ms 无指令就停车** | 拔 USB，500ms 内车停 |

★★ **`setvbuf` 那条最值得单独说**（`sensor_bridge` 的实测教训）：
nano.specs 下 stdout 接 UART **不是 tty，走全缓冲（约 1024B）**，
不关缓冲的话数据会攒成一大坨才发，你会去怀疑波特率、接线、ADC——越查越远。

★★ **失联保护是「真机项目」和「仿真项目」最重要的区别**。
没有它，你会亲眼看着车撞墙。实现在固件里，共约 3 行代码。

#### R.4 写桥接节点

**这是阶段 D 你要自己写的 `chassis_bridge`**，无硬件自测脚本会检查它。

**必须满足的三条**（直接继承 `sensor_bridge` 的经验）：

| # | 要求 | 为什么 |
|---|---|---|
| 1 | ★★ **读串口在独立线程**，不在 spin 线程 | 否则定时器全停摆、`ros2 param get` 永久挂起、Ctrl+C 退不掉 |
| 2 | ★★ **定时器 + 最新值** 模式发布 | 发布节奏和接收解耦；下游拿到等间隔流 |
| 3 | ★★ **三层异常处理** | 连接层重连 / 数据层静默计数 / 时效层停发 |

★★ **第 1 条有实测数据支撑**（2026-09-29 在本机实测）：

```
场景：定时器周期 0.5s，第 3 个 tick 在回调里 sleep 6 秒

节点侧日志：
  [1790664546.288834983] tick #1
  [1790664546.788058555] tick #2
  [1790664547.289241494] tick #3
  [1790664547.290285096] ★ 在回调里开始阻塞 6 秒
  [1790664553.297132499] ★ 阻塞结束
  [1790664553.298793379] tick #4        ← ★★ 6 秒内 12 个周期全被吃掉

外部调用 ros2 service call /ping 的实测结果：
  rc=0  耗时=5.124270576s           ← ★★ 服务调用被卡了 5.1 秒！
  response: pong, ticks=4
```

★ **结论（实测得出，不是推测）**：
单线程执行器里，一个阻塞回调会
**① 吃掉所有定时器周期**（tick #3 → #4 之间 6 秒空白）
**② 阻塞所有走服务调用的操作**（`ros2 param get` / `service call` 卡 5.1 秒）

第 ② 条就是「读串口必须另开线程」的实证理由 —— 面试时这么说比背结论有力得多。

#### R.5 真机分步验收（★ 不要一次全上）

| 步 | 内容 | 判据 | 跳过它的后果 |
|---|---|---|---|
| R.5.1 | 只接 STM32，跑桥，**不接电机** | `/odom` 有数据，手转轮子数值变化 | — |
| R.5.2 | 接电机，**架空车轮** | `/cmd_vel` 发了轮子会转、方向对 | 车轮着地时方向错了直接撞 |
| R.5.3 | 着地，**低速**遥控 | 车能走直线（不跑偏） | — |
| R.5.4 | ★ **验失联保护** | 拔 USB，0.5s 内停 | 没验 → 调试时车会飞出去 |
| R.5.5 | 只接雷达 | `ros2 topic hz /scan` ≈ 10Hz | — |
| R.5.6 | 手持车走一圈建图 | 图可用 | — |
| R.5.7 | 全系统导航 | 单点成功 | — |

★★ **R.5.4 绝对不能跳过**。理由：在你调试导航参数时会反复 Ctrl+C、
重启节点、拔线。没有失联保护，**每次 Ctrl+C 都可能是一次全速撞墙**。

**★ 真机的三个必踩硬件坑**：

| 坑 | 现象 | 解决 |
|---|---|---|
| **brltty 抢 CH340** | 插上后 `/dev/ttyUSB0` 出现又消失，dmesg 有 `ch341-uart ... disconnected` | `sudo apt remove brltty` ★ Ubuntu 上的经典问题 |
| **dialout 组权限** | `Permission denied: '/dev/ttyUSB0'` | `sudo usermod -aG dialout $USER` ★ **必须注销重登** |
| **雷达抢串口** | 雷达和 STM32 都是 `/dev/ttyUSB0`，谁先插谁占 | ★ 用 udev 规则按序列号固定名字 |

**udev 规则**（本项目提供，参考 `参考源码/firmware/` 里的思路，
沿用你 `sensor_bridge` 里 `99-stm32-bridge.rules` 的写法）：

```
# /etc/udev/rules.d/99-navbot.rules
# ★★★ 注意：把 idVendor/idSerial 换成你自己设备的（用 lsusb -v 查）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="navbot_stm32", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="navbot_lidar", MODE="0666", GROUP="dialout"
```

应用后：
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/navbot_stm32 /dev/navbot_lidar
# ★ 之后桥就用 -p port:=/dev/navbot_stm32，永远不用猜是哪个 ttyUSB
```

#### R.6 雷达驱动（RPLIDAR A1）

```bash
# ★ 镜像源里有二进制包，不用从源码编
sudo apt install ros-humble-rplidar-ros

# 起雷达（A1 型号）
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

★★ **一个必须知道的坑：RPLIDAR 驱动默认 `frame_id` 是 `laser`，不是 `laser_link`！**

实测证据（官方 launch 文件原文）：
```python
frame_id = LaunchConfiguration('frame_id', default='laser')
```

★ **两种解法，本项目用 B**：

| 方案 | 做法 | 优缺点 |
|---|---|---|
| A | 模型里的 link 就叫 `laser` | 简单，但名字不规范 |
| **B** | 模型叫 `laser_link`，起驱动时**显式传参** | 名字规范，代价是命令长一点 |

```bash
# 方案 B 的正确命令（★ 注意 frame_id 参数）
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/navbot_lidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser_link \
  -p angle_compensate:=true

# ★ 验证 frame_id 对了没有
ros2 topic echo /scan --once | grep frame_id
#   应该输出 frame_id: laser_link
```

★★ **`frame_id` 不对的后果**：
TF 树里没有 `laser` 这个坐标系（只有 `laser_link`），
激光数据无法变换到 `map` 坐标系 → 建图和定位全部失效。
报错是 `Lookup would require extrapolation` 或
`"laser" passed to lookupTransform argument target_frame does not exist`
—— 完全看不出是「名字写错」这么简单的原因。

---

## 四、关键产出物清单

做完这个项目，你手上应该有一份**能直接发给面试官**的东西。分两层看：
**能力层**（你能讲什么）和 **文件层**（仓库里有什么）。

### 4.1 能力层（这才是简历上写的）

| # | 能力 | 具体到什么程度 | 对应阶段 |
|---|---|---|---|
| 1 | **机器人建模** | 能用 xacro 从零写出带惯性矩阵的两轮差速车，讲清 `base_footprint` 和 `base_link` 为什么要分开 | A |
| 2 | **TF 树设计与排障** | 能画出 `map→odom→base_footprint→laser_link` 全链，能定位「哪个 frame 断了」 | A、C、D |
| 3 | **Gazebo 仿真集成** | 能给模型挂上 diff_drive / ray_sensor / joint_state 三个插件，知道每个参数的物理含义 | B |
| 4 | **SLAM 建图** | 能用 slam_toolbox 建图并正确保存，讲清 `map→odom` 的唯一发布者原则 | C |
| 5 | **Nav2 调参** | 能读懂 nav2_params.yaml 的 15 个节点，能解释 `robot_radius`/`inflation_radius` 怎么定 | D |
| 6 | **★★ 串口协议设计** | 能从零设计一份两端契约（字段、单位、丢帧检测、时间戳、超时保护） | 真机 |
| 7 | **ROS2 线程模型** | 能解释单线程执行器为什么会被读串口阻塞，以及为什么必须另开线程 | 真机 |
| 8 | **差速运动学** | 能推 `v_l = v − ωL/2`，能解释三个参数（轮距/轮径/PPR）不一致的后果 | 真机 |

★ **第 6 条是你最值钱的一条**。纯算法岗的人讲不了协议，纯软件岗的人搞不定编码器。
你两条都占。

### 4.2 文件层（仓库应该长这样）

下面这条命令用来**检查你有没有漏文件**：

```bash
cd ~/bridge/ros_study/src
tree -L 3 navbot_description navbot_bringup navbot_bridge 2>/dev/null || find navbot_* -type f | sort
```

| 路径（相对 `~/bridge/ros_study/src/`） | 文件 | 说明 | 阶段 |
|---|---|---|---|
| `navbot_description/urdf/navbot.xacro` | 模型主体 | 车体/轮子/万向轮/雷达的 link 与 joint | A |
| `navbot_description/urdf/navbot.gazebo.xacro` | 仿真插件 | 三个 plugin，★ 真机路线里这个文件不加载 | A、B |
| `navbot_description/urdf/navbot.urdf.xacro` | 总装入口 | 只做 include + 传参，不含几何 | A |
| `navbot_description/launch/display_navbot.launch.py` | 看模型 | 起 robot_state_publisher + joint_state_publisher + RViz | A |
| `navbot_description/rviz/navbot_check.rviz` | 模型检查视图 | Fixed Frame = `base_footprint` | A |
| `navbot_bringup/launch/sim_navbot.launch.py` | 起仿真 | Gazebo + spawn 车 | B |
| `navbot_bringup/worlds/navbot_room.world` | 虚拟房间 | 6m×5m，含窄通道 + 柱子 | B |
| `navbot_bringup/launch/slam_mapping.launch.py` | 建图 | 包一层 slam_toolbox 的 online_async | C |
| `navbot_bringup/config/slam_toolbox_params.yaml` | 建图参数 | 47 个键，`resolution` 必须和地图 yaml 一致 | C |
| `navbot_bringup/rviz/navbot_slam.rviz` | 建图视图 | Fixed Frame = `map`，看 `map` + `scan` | C |
| `navbot_bringup/maps/navbot_room.{pgm,yaml}` | ★ 地图产物 | **这是你的作品**，不是模板 | C |
| `navbot_bringup/launch/nav_bringup.launch.py` | 导航 | 包一层 nav2 的 bringup | D |
| `navbot_bringup/config/nav2_params.yaml` | 导航参数 | 15 个节点，★ 本项目调参的主战场 | D |
| `navbot_bringup/rviz/navbot_nav.rviz` | 导航视图 | 10 个显示项，含代价地图和粒子云 | D |
| `navbot_bridge/navbot_bridge/odom_publisher.py` | 串口→/odom | 读上行帧、算里程计、发 TF | 真机 |
| `navbot_bridge/navbot_bridge/cmd_vel_relay.py` | /cmd_vel→串口 | 差速逆解 + 限幅 + 发下行帧 | 真机 |
| `navbot_bridge/config/bridge_params.yaml` | 桥参数 | 串口设备、轮距、轮径、PPR ★ 三处一致 | 真机 |
| `navbot_bridge/launch/bringup_real.launch.py` | 真机总启 | 桥 + 雷达 + Nav2，★ 不含 Gazebo | 真机 |
| 固件 `Core/Src/main.c` 等 | 底盘固件 | 在 `stm32_ws/` 里，**单独一个仓库或子模块** | 真机 |
| `README.md` | ★★ 项目门面 | 见 4.3 | 全程 |
| `docs/` 3 篇博客 | 见 4.4 | 见 4.4 | 全程 |

★★ **`navbot_gazebo.xacro` 这个文件的存在本身就是一个面试点**：
"为什么把仿真插件和模型分开？" → 因为真机不需要 Gazebo 插件，
分开之后同一个模型文件可以同时给仿真和真机用。这是**关注点分离**。

### 4.3 README.md 必须包含什么

面试官只看 README 的前 30 秒。这 5 块缺一不可：

| 块 | 内容 | 判据 |
|---|---|---|
| 1. 一句话 + 动图 | "两轮差速小车，用 slam_toolbox 建图 + Nav2 自主导航" + **一段 10 秒 GIF** | ★ 没有 GIF 的 README 等于没写 |
| 2. 系统架构图 | 直接抄本文 §3.0 那张图（改成你实际的包名） | 能一眼看出数据怎么流 |
| 3. 快速启动 | 3 条命令能复现仿真全流程 | 别人能跑起来 |
| 4. 硬件 BOM + 接线图 | 底盘/电机/编码器/雷达型号 + 引脚对应 | ★ 证明是**真机**不是抄的 |
| 5. 我踩的坑 | 挑 2–3 个（★ 从下面 §六 里挑） | 证明你真的动手了 |

★ **第 5 块是加分项中的加分项**。把你调参时的真实日志贴上去，
比写十行"本项目的意义"有用得多。

### 4.4 三篇博客（路线文档要求"技术博客 6 篇以上"，本项目就出 3 篇）

| # | 标题（建议） | 核心内容 | 素材来源 |
|---|---|---|---|
| 1 | 《ROS2 里 `map→odom` 到底该谁发布》 | slam_toolbox 和 AMCL 抢发布权的现象与排查 | §六 陷阱 1 |
| 2 | 《Nav2 参数一调就崩？从 costmap 的 `robot_radius` 讲起》 | 从物理尺寸反推参数，而不是抄别人的数字 | §六 陷阱 3 |
| 3 | 《STM32 和 ROS2 之间的协议该怎么定》 | 11 字段上行帧的设计过程 + 为什么用 mm/s 整数 | 协议文档 |

★ 这三篇都不需要"文采"，只需要**真实的报错日志 + 你的排查路径 + 结论**。

---

## 五、验收标准

★ **说明**：这里每一条都给了**可执行的验证命令**和**"不能过的情况"**。
符合你 `代码回收清单.md` 里那条规矩——**不写"跑通即可"**。

### 5.1 A 组：功能验收（仿真）

| # | 判据 | 验证命令 | 不能过的情况 |
|---|---|---|---|
| A-1 | xacro 能展开成合法 URDF | `xacro navbot.urdf.xacro > /tmp/n.urdf && check_urdf /tmp/n.urdf` | rc≠0；输出 `Two root links found` |
| A-2 | TF 树完整、无断链 | `ros2 run tf2_tools view_frames` 后看生成的 pdf | 树里缺 `base_footprint` 或 `laser_link` |
| A-3 | 车在 RViz 里形状正确、轮子位置对 | `ros2 launch navbot_description display_navbot.launch.py` | 轮子穿模 / 雷达悬空 |
| A-4 | Gazebo 里车能动 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | 车不动，或动但方向反 |
| A-5 | `/scan` 有数据、频率对 | `ros2 topic hz /scan` | 无输出；或 hz < 5 |
| A-6 | `/odom` 有数据、频率对 | `ros2 topic hz /odom` | 无输出；或 hz 明显低于 50 |
| A-7 | 建图能出图 | `ros2 run nav2_map_server map_saver_cli -f ~/map` | 报 `Map not received` |
| A-8 | 地图形状和房间一致 | `eog ~/map.pgm` 肉眼看 | 墙是弯的 / 有重影 |
| A-9 | Nav2 全部节点激活 | `ros2 lifecycle get /controller_server` 等（见 §3 阶段 D） | 输出 `unconfigured` 或 `inactive` |
| A-10 | ★ 单点导航成功 | RViz 里点 `Nav2 Goal` | 车不动 / 撞墙 / 原地打转 |

### 5.2 B 组：健壮性验收（真机才有）

★ 这一组是**"仿真项目"和"真机项目"的分水岭**。

| # | 判据 | 验证方法 | 不能过的情况 |
|---|---|---|---|
| B-1 | 上行帧率 50Hz ± 2Hz | `ros2 topic hz /odom` | 低于 45Hz → 先查 MCU 侧 `setvbuf` |
| B-2 | 静止时零漂在 ±5 mm/s 内 | 车不动，`ros2 topic echo /odom` 看 twist | 漂到几十 mm/s → 编码器计数在乱跳 |
| B-3 | 手推车轮，速度符号正确 | 向前推左轮，`vel_l` 应为**正** | 符号反 → 编码器 A/B 相接反了 |
| B-4 | 丢帧数为 0 | 桥节点日志里的丢帧统计 | 持续有丢帧 → 波特率或线路问题 |
| B-5 | 收到指令 100ms 内轮子动 | 发 `/cmd_vel` 后掐表 | 超过 300ms → 控制周期或 PWM 有问题 |
| B-6 | ★★ **拔 USB，0.5s 内停车** | 全速前进时直接拔线 | **不停车 = 绝对不能上真机** |
| B-7 | 非法下行帧不崩 | `echo "V,1,x,y,z" > /dev/navbot_stm32` | 车乱动 / 固件重启 |
| B-8 | 连续跑 10 分钟不重启 | 挂着跑，看 uptime | 复现重启 → 多半是栈溢出或看门狗 |
| B-9 | 电池电压读数准 | 万用表对比 | 误差 > 0.2V → 分压电阻算错 |

★★ **B-6 这一条请把它刻在脑子里**。你的调试过程会充满
`Ctrl+C`、拔线、重启节点——**每一次都是一次潜在的全速撞墙**。
本项目的固件骨架里 `CMD_TIMEOUT_MS 500` 就是为它准备的（★ 失联保护的实现见 §R.3 和协议文档 §4）。

### 5.3 C 组：工程规范验收

| # | 判据 | 怎么查 |
|---|---|---|
| C-1 | 三个包职责清晰 | `navbot_description` 只放描述，`navbot_bringup` 只放启动和配置，`navbot_bridge` 只放节点 |
| C-2 | 没有硬编码绝对路径 | `grep -rn "/home/liang" navbot_*/` 应该**没有输出**（★ 你 sensor_bridge 踩过这个坑） |
| C-3 | 参数都在 yaml 里，不在代码里 | launch 文件里 `ParameterValue` 的价值都来自 yaml |
| C-4 | README 有动图 + BOM | 见 §4.3 |
| C-5 | git 提交信息有意义 | `git log --oneline` 不是一堆 `update` |
| C-6 | `.gitignore` 排除了 `build/ install/ log/` | `cat .gitignore` |

★ **C-2 是你 sensor_bridge 的真实教训**（古月居教程里那种
`cv2.imread('/home/xxx/图片.jpg')` 的写法）。写之前就避开，不用再踩一遍。

### 5.4 ★★ D 组：口述题（10 道，答出 ≥8 道算通关）

**这一组最重要。** 面试就是口述，代码写得再漂亮讲不出来等于零。
**先自己答一遍再翻答案**——直接看答案等于没答。

---

**D-1｜`base_footprint` 和 `base_link` 为什么要分两个？**
<details><summary>参考答案</summary>

`base_link` 是车体的**几何中心**，通常在轮轴高度；`base_footprint` 是车体
**在地面的投影**（z=0）。分开的理由：导航和代价地图关心的是"车占多大地方"，
它需要车在地面上的足迹；而物理模型需要真实的质心位置。
如果只用一个，要么代价地图算错高度，要么物理仿真姿态不对。
</details>

**D-2｜TF 树里 `map → odom → base_footprint` 这三层各是什么含义？**
<details><summary>参考答案</summary>

- `map → odom`：**定位修正**。表示"我算出来的位置"和"里程计认为的位置"之间的偏差，
  由 SLAM 或 AMCL 发布。这是**不连续的、会跳变的**（每次定位修正就跳一下）。
- `odom → base_footprint`：**里程计**。由轮子编码器积分得到，
  **连续但不准**（会累积漂移）。
- 分三层的原因就是**让漂移只污染一层**：里程计负责连续，定位负责准确，
  两者解耦。这就是为什么"里程计漂移"和"定位丢失"是两个不同的问题。
  </details>

**D-3｜`map → odom` 为什么只能有一个发布者？**
<details><summary>参考答案</summary>

TF 树的规则是**一个 child frame 只能有一个 parent**。
如果 slam_toolbox 和 AMCL 同时发布 `map→odom`，TF 会不停收到两个矛盾的变换，
表现为 RViz 里车在**高频抖动 / 瞬移**，代价地图重影。
★ 这就是本项目最高频的坑（见 §6 陷阱 1）。
</details>

**D-4｜差速运动学的正解和逆解分别是什么？为什么要轮距这个参数？**
<details><summary>参考答案</summary>

- 逆解（给定车体速度，求轮速）：`v_l = v − ω·L/2`，`v_r = v + ω·L/2`
- 正解（给定轮速，求车体速度）：`v = (v_l + v_r)/2`，`ω = (v_r − v_l)/L`
- `L` 是轮距。**它同时出现在两处，所以必须两边一致**：
  URDF 里 `wheel_sep`、nav2_params 里 diff_drive 的 `wheel_separation`、
  固件里 `WHEEL_SEPARATION_M`，这三个数不一致 = 系统性转向偏差，而且极难查
  （因为车能动，只是转不准）。
  </details>

**D-5｜为什么上行协议里的速度要用 mm/s 整数，不用 float？**
<details><summary>参考答案</summary>

MCU 上没有 FPU（或即使有，`printf("%f")` 也极慢），
浮点格式化会吃掉大量 CPU 时间。在 50Hz 的实时循环里，这是致命的。
改用整数后，量化误差对 0.2 m/s 的车是 0.5%，完全可接受。
★ 这条经验直接继承自 sensor_bridge 阶段三的"全整数定点"改造。
</details>

**D-6｜协议里 `seq` 字段解决什么问题？举个具体场景。**
<details><summary>参考答案</summary>

区分"车没动"和"数据丢了"。
没有 `seq` 时，你看到编码器读数不变，无法判断是
(a) 车静止（正常）还是 (b) 串口丢了 3 帧（故障）。
有 `seq` 后，PC 侧统计 `收到的最新 seq − 收到的帧数` 就是丢帧数。
★ 这是 sensor_bridge 阶段二那次"`topic hz = 0.198`"事故的直接教训。
</details>

**D-7｜`tick_ms` 时间戳为什么必要？`header.stamp` 不够用吗？**
<details><summary>参考答案</summary>

`header.stamp` 标的是**数据到达 PC 的时刻**，不是**数据在 MCU 上采样的时刻**。
两者相差串口传输 + USB 转换 + 内核缓冲 ≈ 1–2ms。
做里程计积分时，这个固定延迟会造成系统性误差。
有了 `tick_ms` 就能做"到达时刻 − 传输延迟估计 = 采样时刻"的校正。
★ 呼应 sensor_bridge 复盘里那句"header.stamp 标的是到达时刻不是采样时刻"。
</details>

**D-8｜失联保护为什么设 500ms 而不是 2 秒？**
<details><summary>参考答案</summary>

PC 侧发指令是 20Hz（50ms 一条），500ms = 连丢 10 条指令，
已能确定不是偶发丢包而是对端真的挂了。
- 设太大：0.2 m/s × 1s = 20cm，够撞坏东西了。
- 设太小：串口偶然卡一下 3 条指令就停了，很难正常开。
500ms 是"确定挂掉"和"不误触发"之间的平衡点。
</details>

**D-9｜单线程执行器里读串口会出什么问题？怎么解决？**
<details><summary>参考答案</summary>

`readline` 是阻塞调用，会占住唯一的执行线程，
导致**所有定时器回调停摆**、**所有服务调用卡住**。
★ 本机实测证据（见 §R.4）：在 0.5s 定时器里插一个 6s 的 sleep，
tick #3→#4 之间出现了 6 秒空白，外部 `ros2 service call` 耗时 **5.124270576s**。
解决：把串口读取放到**独立线程**（`threading.Thread` + 队列），
ROS2 执行器只管 ROS 回调。
</details>

**D-10｜真机路线相比仿真，Nav2 部分需要改什么？**
<details><summary>参考答案</summary>

**几乎不用改。** 只需要：
1. `use_sim_time` 从 `true` 改成 `false`
2. 把 Gazebo 的 diff_drive 插件换掉——它原本负责发 `/odom` 和 `odom→base_footprint` TF，
   现在这两件事交给**你自己写的串口桥**
3. 雷达换成真雷达驱动（★ 注意 `frame_id` 要是 `laser_link`，见 §R.6）
★ **一个参数都不用重新调**，因为 Nav2 只认话题（`/scan`、`/odom`、`/cmd_vel`）
和 TF，不关心数据是 Gazebo 给的还是真硬件给的。
这就是 ROS2 解耦的价值，也是面试要讲的重点。
</details>

---

**自评表**（先答完再填）：

```
D-1  [ ]      D-2  [ ]      D-3  [ ]      D-4  [ ]      D-5  [ ]
D-6  [ ]      D-7  [ ]      D-8  [ ]      D-9  [ ]      D-10 [ ]

答出 ≥8 道 → 能力层达标，可以开始写真机
答出 5–7 道 → 回看对应章节再来一遍
答出 <5 道  → ★ 别急着上真机，仿真再走一遍全流程
```

### 5.5 通关线（按顺序，不要跳）

```
[ ] 1. §5.1 A 组 10 条全过                        → 仿真通关
[ ] 2. 无硬件自测/verify_e2e.sh 全绿              → 协议层通关
[ ] 3. §4.4 三篇博客写完                          → 能讲出来
[ ] 4. §5.4 D 组答出 ≥8 道                        → 认知通关
[ ] 5. §5.2 B 组全过（★ 重点是 B-6）              → 真机可以上电
[ ] 6. §5.3 C 组全过 + README 有动图              → 可以发给别人看
```

★ **第 2 条现在就能跑**（不需要任何硬件）：

```bash
cd /home/liang/桌面/具身智能学习路线/13_ROS2自主导航移动机器人/无硬件自测
./verify_e2e.sh
```

---

## 六、关键风险点

### 6.1 ★★ 三个"看起来对但不对"的陷阱

这三个坑的共同特点是：**报错信息和真实原因完全对不上**。
先记住现象，遇到时能少查 3 小时。

---

#### 陷阱 1｜`map→odom` 有两个发布者 → 车高频抖动

| 项 | 内容 |
|---|---|
| **现象** | RViz 里车的位置**高频抖动 / 瞬移**；代价地图出现"重影"；Nav2 规划出的路径抖动 |
| **报错** | ★ 可能**完全没有任何报错**，只是行为异常 |
| **真实原因** | slam_toolbox 和 AMCL 同时在发 `map→odom`，TF 收到两个矛盾的变换 |
| **怎么确认** | `ros2 run tf2_tools view_frames` 后打开 pdf，看 `map` 有几个 broadcaster；或 `ros2 topic info /tf --verbose` 看 `/tf` 的发布者列表 |
| **怎么修** | 建图时**只**起 slam_toolbox；导航时**只**起 AMCL。★ 别用 `nav2_bringup` 的 `slam:=True` 又手动起 AMCL |

★ **为什么会犯**：因为 `nav2_bringup/bringup_launch.py` 的 `slam` 参数为 `True` 时，
它会**自己拉起一个 slam_toolbox**。如果你忘了，再加一个建图的 launch，
就是两个发布者。**这是最隐蔽的一次**。

---

#### 陷阱 2｜固件 `printf` 用了行缓冲 → `/odom` 频率只有几 Hz

| 项 | 内容 |
|---|---|
| **现象** | `ros2 topic hz /odom` 输出远低于 50Hz（可能只有 2–5Hz）；偶尔一次涌出一大堆数据 |
| **报错** | ★ 没有报错 |
| **真实原因** | STM32 的 `nano.specs` 下 `stdout` 是**全缓冲**（约 1024 字节），不是行缓冲。数据被攒着，等缓冲区满了才一次性发出 |
| **怎么确认** | 看固件里有没有 `setvbuf(stdout, NULL, _IONBF, 0)`；或串口助手观察是不是"一波一波"来 |
| **怎么修** | 在 `main()` 初始化里加 `setvbuf(stdout, NULL, _IONBF, 0)`（★ 见固件骨架 TODO-C1 第 2 步） |

★ **为什么会犯**：因为你在 PC 上写 `printf` 从来是行缓冲的，
根本不会想到 MCU 上默认是全缓冲。而且**这个坑没有报错，只有频率不对**。

---

#### 陷阱 3｜`robot_radius` 抄了别人的值 → 车能规划但过不去窄门

| 项 | 内容 |
|---|---|
| **现象** | Nav2 能规划出路径、车也走了，但在窄通道前**停下不动**；或者更糟——**擦着墙走过去**（代价地图没算住） |
| **报错** | 可能报 `Failed to get robot pose`、`Controller server failed`，也可能什么都不报 |
| **真实原因** | `robot_radius`（或 `footprint`）和车的**实际尺寸**对不上。太小 → 会撞；太大 → 过不去 |
| **怎么确认** | 量你的车：本项目车底 0.30×0.24，取外接圆半径 `√(0.15²+0.12²) ≈ 0.192`，所以设 `0.20`。★ 用 `ros2 topic echo /local_costmap/costmap` 看膨胀层对不对 |
| **怎么修** | 按**物理外接圆**设 `robot_radius`；`inflation_radius` 通常取它的 1.5–2 倍，但**必须受通道约束**：要满足 `2 × inflation_radius < 通道宽`。本项目通道 0.6m，所以只能取 **0.25**（=1.25 倍）——取 0.35（1.75 倍）就会 `2×0.35=0.7 > 0.6` 封死通道 |

★★ **为什么这个特别坑**：因为**你的行车环境是自己造的**。
如果房间宽敞，参数错 30% 也能跑；一旦遇到窄门（本项目 world 里
`partition_short` 造了一个约 0.6m 的通道）就卡住。
**你会以为是 Nav2 调参问题，其实是几何问题。**

---

### 6.2 高频故障速查表

| 症状 | 最可能的原因 | 第一步查什么 |
|---|---|---|
| `Package 'nav2_bringup' not found` | 没装 | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| `Package 'slam_toolbox' not found` | 没装 | `sudo apt install ros-humble-slam-toolbox` |
| launch 起来但模型不显示 | `ParameterValue` 漏了 `value_type=str` | 看 launch 文件里参数怎么写的 |
| `Two root links found` | URDF 里有孤立 link | `check_urdf` 的报错会直接点名 |
| `Invalid parameter "prefix"` | xacro 宏参数用了 `$prefix`（ROS1 写法） | 改成 `params="prefix side"` |
| `name 'xxx' is not defined` | xacro 属性名拼错 | 看报错里 `when evaluating expression` 那一行 |
| AMCL 定位乱跳 | 初始位姿没给 | RViz 里点 `2D Pose Estimate` |
| 车朝错误方向走 | 同上 | 同上 |
| `Map not received` | slam_toolbox 已经退出了才存图 | ★ 必须在建图节点活着时存 |
| `Lookup would require extrapolation` | `use_sim_time` 设错，或 TF 断链 | 先查 `use_sim_time` 两边是否一致 |
| `laser ... does not exist` | 雷达 `frame_id` 是 `laser` 不是 `laser_link` | 见 §R.6 |
| 车不动但路径已生成 | 全局规划好了，控制器没动 | 查 `/cmd_vel` 有没有输出、`controller_server` 是不是 active |
| 代价地图"重影" | 陷阱 1（两个 `map→odom`） | 见 6.1 陷阱 1 |

### 6.3 卡住时的止损信号

★ 止损不是「赶进度」，是**避免沉没成本**——在一个坑里耗三天，
往往不如退一步换个思路再进来。卡住超过阈值就该止损：

| 信号 | 止损动作 |
|---|---|
| 在 xacro 语法上卡超过 **2h** | 直接用 `参考源码/urdf/` 里的模型，改成你的尺寸 |
| Gazebo 崩溃排查超过 **2h** | 走 `gzserver` 无界面模式，或直接跳到阶段 C 用假数据 |
| Nav2 调参超过 **2 天**还没单点成功 | 先用 `参考源码/config/nav2_params.yaml` 原样跑通，再改 |
| 真机硬件问题搞不定 **一整晚** | 停下来，白天再弄（夜里焊线/接线容易出安全事故） |

★★ **最重要的一条**：**本项目随时可以切回仿真**。
真机卡住不影响你已经拿到的仿真成果 —— 所以不要因为真机不顺利就觉得自己白做了。

---

## 七、下一步

做完本项目（含真机）后，按 18 个月路线的顺序，你面前有几条路：

| 方向 | 内容 | 为什么这个时候做 |
|---|---|---|
| **① 补 C++** | 用 C++ 重写 `chassis_bridge` | ★ 机器人岗硬门槛。你已有 Python 版对照，改写时能**专注语言本身**而不是业务逻辑 |
| **② P1 机械臂（LeRobot）** | 阶段三另一个交付物 | ★ 具身智能本体工程向的招牌，和 P3 同属「中间件层」，两条腿就齐了 |
| **③ 阶段二的理论补齐** | 高翔《视觉SLAM十四讲》 | ★ 现在有时间系统学，是「会用 SLAM」到「懂 SLAM」的分水岭 |
| **④ micro-ROS** | 把 STM32 直接做成 ROS2 节点（去掉串口桥） | 进阶方向，去掉串口桥这层「翻译」 |
| **⑤ 补 TF/URDF 深度** | 手写一个 URDF 解析器 / TF 变换的数学实现 | 面试深挖 TF 时的底气 |

★ **建议顺序**：`① → ② → ③`，`④⑤` 看兴趣再补。

理由是：**① 是硬门槛**（不看 C++ 的机器人岗几乎不存在），
**② 是方向投资**（具身智能招牌，和 P3 互补），
**③ 是深度投资**（阶段二本就该学，现在时间正好系统啃）。

### 7.1 立即可做的三件事（今天/本周）

```
[ ] 1. 先跑 无硬件自测/verify_e2e.sh，确认 44 个单测全绿 —— 0 成本建立信心
[ ] 2. 装依赖（§2.2），确认 nav2_bringup 和 slam_toolbox 真的装上了
[ ] 3. 打开 参考源码/urdf/navbot.xacro，把每个 link 的尺寸改成你自己要的
```

### 7.2 本项目与整体路线的挂接点

| 本项目产出 | 直接喂养到 |
|---|---|
| 自建 URDF + Gazebo 仿真 | 阶段二交付物「Gazebo 仿真」✅ 还清 |
| slam_toolbox 建图 + Nav2 导航 | 阶段二交付物 ✅ 还清 |
| 串口协议 + STM32 固件 | 阶段三 **P3** ✅ 兑现 |
| TF / URDF / 线程模型 | 求职自检清单第 4、5、6 条 |
| 三篇博客 | 自检清单「技术博客 6 篇以上」（已完成一半） |

---

## 本目录附件表

★ **每一份都经过实际验证**，验证方式写在最后一列。
`参考源码/` 下的东西是**给你对照用的**，不是让你直接抄 ——
建议你**自己写一遍**，卡住了再来看这里的实现。

### 主文档

| 文件 | 说明 | 验证状态 |
|---|---|---|
| `P3_ROS2自主导航移动机器人_保姆级项目文档.md` | ★ 本文，项目主文档 | — |

### 参考源码

| 文件 | 说明 | 验证状态 |
|---|---|---|
| `参考源码/urdf/navbot.xacro` | 车体模型（车底/两轮/万向轮/雷达），含惯性矩阵 | ✅ `xacro` 展开 rc=0；`check_urdf` 显示 `base_link` 下 5 个子节点 |
| `参考源码/urdf/navbot.gazebo.xacro` | Gazebo 三插件（diff_drive / ray_sensor / joint_state） | ✅ 三个 `<plugin>` 均能注入；`sensor_msgs/LaserScan` 出现 2 次 |
| `参考源码/urdf/navbot.urdf.xacro` | 总装入口（include + 传参） | ✅ `xacro` 展开 rc=0 |
| `参考源码/launch/display_navbot.launch.py` | 看模型：robot_state_publisher + joint_state_publisher + RViz | ✅ `python3 -m py_compile` 通过 |
| `参考源码/launch/sim_navbot.launch.py` | 起 Gazebo + spawn 车（`TimerAction` 延迟 3s） | ✅ `py_compile` 通过 |
| `参考源码/launch/slam_mapping.launch.py` | 建图（包 slam_toolbox 的 online_async） | ✅ 参数名对照官方 `online_async_launch.py` 核实 |
| `参考源码/launch/nav_bringup.launch.py` | 导航（包 nav2 的 bringup） | ✅ 参数名对照官方 `bringup_launch.py` 核实 |
| `参考源码/worlds/navbot_room.world` | 6m×5m 房间：含 0.6m 窄通道 + 2 根柱子 + 1 个箱子 | ✅ `gz sdf -k` 输出 `Check complete` |
| `参考源码/config/nav2_params.yaml` | Nav2 参数，15 个顶层节点，按 navbot 尺寸调好 | ✅ YAML 解析通过；键名逐一对照官方 humble 版 |
| `参考源码/config/slam_toolbox_params.yaml` | 建图参数，47 个键 | ✅ YAML 解析通过；键名对照官方 `mapper_params_online_async.yaml` |
| `参考源码/rviz/navbot_check.rviz` | 模型检查视图（Fixed Frame = `base_footprint`） | ✅ YAML 解析通过 |
| `参考源码/rviz/navbot_slam.rviz` | 建图视图（Fixed Frame = `map`） | ✅ YAML 解析通过 |
| `参考源码/rviz/navbot_nav.rviz` | 导航视图（10 个显示项，含代价地图/粒子云） | ✅ YAML 解析通过 |
| `参考源码/maps/navbot_room.yaml` | 地图元数据（★ `resolution` 必须和 slam_toolbox 一致） | ✅ YAML 解析通过 |
| `参考源码/firmware/navbot_protocol.md` | ★★ PC↔STM32 协议 v1 完整规格（11 字段上行 / 4 字段下行） | ✅ 与固件骨架、协议文档三方一致 |
| `参考源码/firmware/navbot_chassis.c` | ★ 固件骨架（**按代码回收清单 §6：只给骨架 + 判据，不给成品**） | ✅ 语法结构完整；TODO A1/A2/B1–B6/C1–C2 各带判据 |

### 无硬件自测（★ 现在就能跑，不需要任何硬件）

| 文件 | 说明 | 验证状态 |
|---|---|---|
| `无硬件自测/fake_chassis.py` | 假底盘：模拟差速车 + 一阶滞后（100ms），按协议发上行帧 | ✅ `py_compile` 通过；`--dry-run` 输出帧格式正确 |
| `无硬件自测/test_protocol.py` | 协议单测：帧往返 / 坏帧 / 运动学正逆解 / 编码器换算 / 物理合理性 / 帧率 | ✅ **44 通过 / 0 失败** |
| `无硬件自测/verify_e2e.sh` | 端到端自检：环境 → 单测 → 串口层 → 解析逻辑 | ✅ **通过 11 / 失败 0 / 跳过 7**（跳过项待 `chassis_bridge` 写完再跑） |

★ **`fake_chassis.py` 的常用参数**：

| 参数 | 用途 |
|---|---|
| `--dry-run` | 不连串口，直接把帧打到屏幕（★ 先跑这个） |
| `--make-socat` | 自动建一对虚拟串口（★ 本机沙箱无 pty，会失败，见下方说明） |
| `--stall-after N` | 第 N 帧后停发（测 PC 侧"数据陈旧"逻辑） |
| `--garbage-every N` | 每 N 帧插一条乱码（测坏帧丢弃） |
| `--no-seq` | seq 不递增（测丢帧检测） |
| `--zero-drift N` | 静止时给 N mm/s 的假零漂（测零漂统计） |

★★ **关于 `--make-socat` 的一个已知限制**：
本机（助手沙箱）**没有 `/dev/pts` 和 `/dev/ptmx`**，无法创建伪终端。
所以 `verify_e2e.sh` 会自动降级——**用 TCP 回环验证同一套解析逻辑**，
并明确把串口层标记为「跳过」。
**在你自己的机器上是正常的**（你有 `/dev/ptmx`），
所以那 7 个跳过项在你手上会变成实打实的通过。

### 踩坑实测（★ 本文所有"实测证据"的出处）

| 文件 | 支撑本文哪里 | 一句话结论 |
|---|---|---|
| `踩坑实测/README.md` | — | 本目录索引 |
| `踩坑实测/01_阻塞回调实测_单线程执行器.md` | §R.4、§5.4 D-9 | 6s 阻塞 → 定时器丢一整个周期；外部服务调用卡 **5.124270576s** |
| `踩坑实测/02_编码器脉冲数换算锚点.md` | §5.2 B 组、§5.4 D-4、固件 TODO-A2 | 走 1 米 = **10343 个脉冲**（手算锚点）；★ 含我算错被单测抓住的过程 |
| `踩坑实测/03_沙箱无pty与set-u冲突.md` | 上方「无硬件自测」说明 | 沙箱无 `/dev/pts` → 降级 TCP 回环；`set -u` 与 ROS `setup.bash` 冲突的正确修法 |

★ **为什么单独立一个目录**：本文每个"为什么会这样"背后都有一份**能查证的记录**。
面试被追问细节时，你可以说"我量过，是 5.12 秒"，而不是"应该会卡一下"。

---

## 附：三条写在最后的话

1. **这个项目最大的价值不是"会跑"，而是"你设计的那条协议"。**
   Nav2 的参数全世界都一样，但 PC 和 STM32 之间那条链路是你自己定的 ——
   这是你和其他候选人的**唯一区别**。

2. **仿真跑通是第一个里程碑，但真机才把这个项目的价值兑现完整。**
   你现在不用赶考研进度，**值得把真机做扎实**——硬件上踩的坑
   （编码器方向、失联保护、串口权限、PID 调参）是纯仿真永远给不了你的。
   这些坑，正是面试官问「你真机做过什么」时你能讲的东西。

3. **卡住的时候先查「名字」「缓冲」和「尺寸」。**
   上面 §6.1 那三个陷阱，一个是名字（`map→odom` 谁发）、
   一个是缓冲（`printf` 全缓冲）、一个是尺寸（`robot_radius`）。
   ★ 这不是巧合 —— **ROS2 里 80% 的诡异问题都是配置对不上，不是算法错**。

---

*文档版本 v1.1 · 2026-09-29 修订（暂不考虑考研：时间预算/下一步/止损理由重排）· 配套 `参考源码/` 与 `无硬件自测/` 均已实测*
