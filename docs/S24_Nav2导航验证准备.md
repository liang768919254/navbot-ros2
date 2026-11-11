# S24 Nav2 导航验证：准备与巡检脚本

日期：2026-11-10
上游：S22（闭环修复）、S23（建图质量量化）

---

## 一、地图已更新

旧地图 `navbot_room.pgm` 建于 **11-09 21:22**，是**修复前**用不闭合路线建的图。
用 `map_quality.py --save` 把当前这张（修复后跑的闭环）存成新图并替换。

| | 旧图 (11-09) | 新图 (11-10) |
|---|---|---|
| 未知区 | 2.5% | **2.2%** |
| wall_west 偏差 | +0.045 | **+0.042** |
| wall_east 偏差 | +0.045 | **+0.042** |
| wall_south 偏差 | +0.025 | +0.030 |
| wall_north 偏差 | +0.025 | +0.030 |
| origin | [−2.980, −2.500] | [−2.983, −2.495] |

两张图都验过「回读 PGM → 按 yaml 还原世界坐标 → 重测墙体」，
位置全部正确，**PGM 行序没有颠倒**。

旧图已备份到 `maps/.bak/navbot_room_20261110_before.{pgm,yaml}`。

### 保存地图踩的三个格式坑

`map_quality.py --save` 第一版有三个问题，都修了：

1. **`image:` 写了绝对路径** —— 装到 `install/share` 后就失效，仓库里也不该出现本机路径。改成相对文件名。
2. **`free_thresh: 0.196`** —— 与仓库现有地图的 `0.25` 不一致。同一份 PGM 用不同阈值判读，两版地图没法对比。
3. **复制顺序错**：先把 yaml 复制成 `navbot_room.yaml` 再删 `navbot_room_new.pgm`，
   导致 yaml 里的 `image:` 指向已删除的文件。**改文件名必须先改引用再删文件**。

另外 PGM 行序是**自顶向下**、占据栅格是**自底向上**，写文件必须
`for iy in reversed(range(h))`。这个错了在 RViz 里看起来像「地图整体歪了」，极难定位，
所以统一用「写完回读验证」来兜底。

---

## 二、当前处于建图模式，需手动切导航

实测 ROS 图里只有 `slam_toolbox`，没有 `amcl`：

```
/diff_drive  /gazebo  /joint_state_publisher  /laser_controller
/robot_state_publisher  /rviz  /slam_toolbox
```

`map→odom` 只能有一个发布者：建图时是 slam_toolbox，导航时是 AMCL。
两个同时开会 TF 抖动、位置乱跳，所以我不能直接叠一个 Nav2 上去。

**切换命令（在你的终端跑）：**

```bash
# 先 Ctrl+C 停掉当前 live.sh，然后：
cd ~/navbot_ws/tools/verify/sim_fullchain
bash live.sh nav
```

`live.sh nav` 会起 Gazebo + `nav_bringup.launch.py`（AMCL + Nav2，`slam:=False`）+ RViz。

---

## 三、目标点净空预检（新增）

导航验证最容易犯的错是「把目标点设在障碍物里」，然后误判为规划器有问题。
先用世界真值算一遍净空（判据 ≥ `robot_radius` 0.20，建议 ≥0.30 留余量）：

| 目标点 | 净空 | 最近障碍 | 结论 |
|---|---|---|---|
| (1.5, 1.0) 原文默认 | 0.381 | pillar_a | 可用 |
| (2.20, −1.50) 东南角 | 0.750 | 外墙 | 可用 |
| (2.20, 1.90) 东北角 | 0.354 | pillar_a | 可用（偏紧） |
| (−2.20, 1.90) 西北角 | 0.550 | 外墙 | 可用 |
| **(−2.20, −1.50)** | **0.150** | pillar_b | **不可达，别用** |
| (0.00, ±2.00) 南北走廊 | 0.450 | 外墙 | 可用 |
| (0.90, −0.10) | 0.250 | partition_short | 偏紧 |
| **(0.90, 0.50) 窄道中点** | **0.100** | partition_mid | **不可达，别用** |
| (0.70, −0.10) 窄道口 | 0.450 | partition_short | 可用 |

> 窄道只能用「**穿过**」的方式测，不能把目标点设在窄道内部。

---

## 四、新增 `nav_patrol.py`

`verify_nav.sh` 只测单个点，测不出「能不能穿 0.6 m 窄道」这个真正的考点。
`nav_patrol.py` 跑一条 7 点序列：

```
东南角 → 南走廊 → 西北角 → 北走廊 → 东北角 → 窄道前南侧 → 窄道后北侧
                                    └── 后两点强制穿越 0.6 m 通道 ──┘
```

每点判据：
- ① action 终态 = `SUCCEEDED`
- ② 到点误差 < 0.25 m
- ③ 用时 < 120 s

用法：

```bash
source ~/navbot_ws/tools/verify/sim_fullchain/env.sh
python3 nav_patrol.py                 # 全部 7 点
python3 nav_patrol.py --no-narrow     # 跳过窄道点
python3 nav_patrol.py --one 2.2 -1.5  # 单点
```

`parse_result` 做了 4 组输入自测（SUCCEEDED / ABORTED / 只 accept 无终态 /
空输出），确认不会把「接受了但超时」误判成成功。

---

## 五、判据边界

- 净空是按**世界真值**算的静态几何，没算代价地图膨胀后的实际可通行宽度。
  膨胀半径 0.25 时，0.6 m 通道两侧各膨胀 0.25 → 只剩 0.10 m 名义自由区，
  这是**刻意设计的考点**，不是 bug。`docs/SAFETY_LIMITS.md` 有推导。
- `nav_patrol.py` 的误差读的是 `/amcl_pose`，即**定位估计值**，不是 `/odom`。
  两者在正常工作时应当一致；若差异 > 0.1 m 说明 AMCL 有漂移，需要单独看。
- 没验证动态障碍（`movable_box` 被推到路上时 DWB 的绕行行为），那是另一组用例。
