# P1 节点层实现说明

> 实现时间：2026-11-17（AI 代写，用户明确要求「剩下的活也全做了」）
> 文件：`src/chassis_node.cpp`（约 380 行）+ `include/navbot_chassis/chassis_node.hpp`
> 目标：用 rclcpp 重写 `navbot_bridge/chassis_bridge.py`（448 行），行为逐字段一致

---

## 一、三条硬要求怎么落实的

Python 版开头写死了 R1/R2/R3，C++ 版照抄这三个骨架：

| 要求 | 落实方式 | 代码位置 |
|---|---|---|
| **R1** 读串口独立线程 | `std::thread read_thread_`，只做「字节 → FrameReader → on_frame」 | `read_loop()` |
| **R2** 定时器 + 最新值 | `create_wall_timer` 50 Hz，快照 `latest_` 后发布，绝不在串口上阻塞 | `on_publish()` |
| **R3** 三层异常 | 连接层重连 / 数据层计数 / 时效层暂停 | 见下表 |

### R3 三层异常的分工

| 层 | 触发条件 | 行为 | 关键代码 |
|---|---|---|---|
| **连接层** | `open()` 失败 ／ `read()` 返回非 EAGAIN 错误 | 等 1 s 重试；**绝不抛异常退出**（否则 systemd 判崩溃） | `read_loop()` 的 `continue` / `break` |
| **数据层** | 坏帧（帧头错/字段少/数字非法） | 只进 `FrameReader::bad_frame_count()`，**一条日志都不打** | 不在 `on_frame` 里处理 |
| **时效层** | 超 `data_timeout_s`（默认 0.5 s）无新帧 | **暂停发布 `/odom`**，只警告一次 | `on_publish()` 开头 |

> **为什么时效层不能发旧值**：Nav2 会拿旧位姿继续规划，机器人"以为自己在原地"，
> 实际车已经滑走 —— 比停下来更危险。

---

## 二、串口为什么用 termios 而不是 serial_driver

本机没装 `ros2 serial_driver`（也没有 asio），所以直接用 POSIX `termios`：

```cpp
::open(port, O_RDWR | O_NOCTTY);   // O_NOCTTY：不让串口成为控制终端
cfmakeraw(&tty);                   // 原始模式，不做任何字符转换
cfsetispeed/ospeed(&tty, B115200);
tty.c_cc[VMIN] = 0;                // 不要求最小字节数
tty.c_cc[VTIME] = 1;               // 1×100ms 读超时 → read 最多阻塞 100ms
```

**`VTIME=1` 这个设计是刻意的**：`read()` 阻塞但最多 100 ms，
于是退出时不需要靠信号打断系统调用，`closing_` 标志下一轮就生效。

`O_NOCTTY` 也不能省：否则 Ctrl+C 的 SIGINT 会先发给串口设备而不是本进程。

---

## 三、里程计积分：中点法

```cpp
const double d_center = (d_l + d_r) / 2.0;
const double d_yaw    = (d_r - d_l) / wheel_separation_;
const double yaw_mid  = yaw_ + d_yaw / 2.0;      // ★ 半个增量
x_ += d_center * std::cos(yaw_mid);
y_ += d_center * std::sin(yaw_mid);
yaw_ += d_yaw;
```

用 `yaw_mid` 而不是 `yaw_`：直行时两者等价，**转向时欧拉法会系统性外扩**（弧线被当成折线）。
这是差速里程计最小的一个精度改进，一行代码。

反馈速度用 MCU 报的 `vel_l/vel_r` 而不是编码器差分 —— 后者在低速时噪声大。

---

## 四、一处必须你确认的矛盾

| 来源 | 字段数规则 |
|---|---|
| **协议文档 §8.3** | 「12 个字段 → **接受**，忽略第 12 个（§7 扩展预留）」 |
| **Python 版 `parse_odom_frame`** | 「字段数**严格等于 11**，多一个也算坏帧」 |
| **本 C++ 实现** | 跟协议文档（接受 >11，忽略多余） |

判定依据：协议文档 §8.4 自己写的「样例表是**唯一来源**，代码里的注释不算」。

**如果要跟 Python 版对齐**：
1. 改 `protocol.cpp` 的 `parse_odom_frame`，把 `n < 11` 改成 `n != 11`
2. 同步改 `test_protocol.cpp:82` 的 `AcceptsExtraFieldsAndIgnoresThem` 用例

---

## 五、验证状态（诚实标注）

| 项 | 状态 | 说明 |
|---|---|---|
| 编译 | ✅ | 零警告（`-Wall -Wextra -Wpedantic`） |
| 协议层单测 | ✅ | 15 / 15 全绿 |
| 连接层：设备不存在不崩 | ✅ | 跑 12 s 存活，告警间隔精确 5 s（节流生效），日志 5 行 |
| 数据层：坏帧不刷屏 | ✅ | 端到端跑完 0 条 WARN/ERROR，日志 3 行 |
| 时效层：超时停发 | ✅ | 启动瞬间误报已修（见 §4.1），实际断流才告警 |
| **端到端** | ✅ | **`/odom` 49.996 Hz、`/joint_states` 49.996 Hz、TF `odom→base_footprint` 闭合** |

### 4.1 实测抓到一个真 bug（已修）

首次端到端跑，日志里这两行只差 **20 毫秒**：

```
083.728083428  串口已打开：/tmp/ttyV1
083.748...     超过 0.500000s 没有新帧，暂停发布 /odom
```

**根因**：`last_recv_t_` 默认构造为 `steady_clock::time_point{}`（epoch），
从未收到帧时 `now - recv_t` 是天文数字，直接命中超时分支。
原代码把「从未收到过」和「收到过但陈旧」混在一个 `if` 里判了。

**修法**：拆成两段 —— `!has` 打 INFO「等待第一帧」；`has && age_s > timeout` 才报超时。

> 这类 bug 的通病：**初始状态**没定义好。默认构造的 time_point、0 值时间戳、
> 未初始化的标志位，都是同一个坑。写时序代码时先问一句「第一帧到来之前，这些值是什么」。


### 端到端验证怎么跑（在你的普通终端里，不走沙箱）

```bash
~/navbot_ws/tools/verify/chassis_cpp_e2e.sh
```

它会自动：建虚拟串口 → 起假底盘（50 Hz）→ 起 C++ 节点 → 检查
`/odom` 数据、频率、`/joint_states`、`odom→base_footprint` TF、日志行数。

**预期**：`/odom` 有数据、频率接近 50 Hz、TF 树闭合、日志 < 500 行。

### ★ 首次跑失败的坑：两端抢同一个 pty（2026-11-17 实测）

第一次跑的现象：

```
[INFO] 串口已打开：/tmp/ttyV0 @ 115200
[WARN] 超过 0.500000s 没有新帧，暂停发布 /odom
✘ /odom 没数据
```

但假底盘日志显示它已经发了 650+ 帧。**串口通了，数据却没到节点。**

**根因**：`fake_chassis.py --make-socat` 建完 pty 对后，自己把 `args.port` 设成了
`/tmp/ttyV0`（源码里那行 `args.port = "/tmp/ttyV0"`）。如果桥节点也连 `ttyV0`，
两个进程打开的是**同一个 slave 端**，假底盘写进去的数据被 socat 从 master 转走，
节点这条读路径上是空的。

**正确接法**：假底盘占 `ttyV0`，桥节点连**另一端** `ttyV1`。

```
ttyV0 (假底盘写) → pty A master → socat → pty B master → ttyV1 (节点读)   ✔
                                                       ↖ 节点写 V 帧，反向同样通
```

`chassis_cpp_e2e.sh` 已按此修正（`-p port:=/tmp/ttyV1`）。

**以后自己写验证脚本时记住**：pty 对的两端不能给同一个进程用。

### 故障注入（验证三层异常）

```bash
# 数据层：每 20 帧插一帧乱码 → 节点不崩，日志不爆
python3 tools/verify/fake_chassis.py --make-socat --garbage-every 20

# 时效层：发 100 帧后静默 → /odom 应在 0.5 s 内停止发布，只警告一次
python3 tools/verify/fake_chassis.py --make-socat --stall-after 100
```

---

## 六、编译时踩到的一个环境坑

```
fatal error: error writing to /tmp/ccEefT6U.s: 设备上没有空间
```

**根因**：本机 `/tmp` 是 10 MB 的 tmpfs，g++ 写临时汇编文件放不下（`/` 还有 65 G）。

**解法**：

```bash
export TMPDIR=$HOME/.cache/gcc-tmp
colcon build --packages-select navbot_chassis
```

这不是代码问题，换台机器可能不出现。但如果哪天又报「没有空间」，先 `df -h /tmp`。

---

## 七、待办（按优先级）

| # | 事项 | 为什么 |
|---|---|---|
| 1 | **端到端跑一次** `tools/verify/chassis_cpp_e2e.sh` | 上面 ❌ 那三项要靠它才能填上 |
| 2 | 接真机（STM32 那 25 个 TODO 填完后） | 真机才是最终判据 |
| 3 | **闭卷重写协议层** | 见 `26_岗位缺口补齐/02_Cpp单元测试gtest/四关作战地图.md` 的「闭卷重写清单」 |
| 4 | 把 `param` 加进 launch / yaml | 现在只能命令行 `-p` 传 |

> 第 3 条是最要紧的：节点层是 I/O 编排，讲得清就行；
> **协议层是你简历上「C++ 实战」唯一能站住的东西**，代写了必须自己补回来。
