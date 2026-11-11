# S28 复盘：原地转向时，DWB 是瞎的

> ## ⚠️ 勘误（2026-11-10 晚，S29 之后追加）
>
> 这篇的**病因分析（第三节）完全成立**，但**第五、六节的配置写法有三处错**，
> 照着抄会得到一个 `[FATAL] + SIGSEGV` 的 controller_server。修正如下：
>
> | 本文原写法 | 正确写法 |
> |---|---|
> | `primary_controller: "FollowPathDWB"` | `primary_controller: "dwb_core::DWBLocalPlanner"`（**类名**，不是参数块名） |
> | 另开 `FollowPathDWB:` 块放 DWB 参数 | DWB 参数**全部写在 `FollowPath:` 下**（内层读 `plugin_name + "."`） |
> | `rotate_to_heading_when_stopped: true` | Humble **没有这个参数**，删掉（原地转由 `linear` 恒 0 天然保证） |
>
> 详细推导见 [`S29_检查脚本全绿仿真却起不来.md`](S29_检查脚本全绿仿真却起不来.md)。
> 本文保留原文不改，用来对照「当时的推理哪里对、哪里想当然了」。
>
> 另：本文第八节说 `check_controller.py` 有 9 项检查——那 9 项当时的判据方向是错的
> （查「yaml 里有没有这个 key」，而 key 是自己起的）。现为 11 项，核心改成
> `strings` 扫 `.so` 比对 pluginlib 真实类名。
>

> 现象：8 点巡检，第 1 点（东南角）从原点走 1.75 m 后卡死，177 秒 `ABORTED`。
> 车停在 `(0.70, −1.58)`，目标 `(2.20, −1.50)` 就在**正东 1.50 m**。
> 用户截图：车（红菱形）停在短板西侧，绿线（全局路径）从车附近往南绕出去。
> 用户原话：**「还是不行」**
>
> ★ 这篇接 S27。S27 修的是「缝 N 被膨胀封死」，这次的问题**跟缝宽无关**，
> 是控制器本身的盲区。

---

## 一、先证明「物理上完全能走」

诊断第一步永远是**排除物理不可能**，不然容易对着一个无解的局面调参数。

| 检查项 | 数值 | 判定 |
|---|---|---|
| 车在 `(0.70,−1.58)` 的净空 | 0.417 m | 够（车半径 0.20） |
| 原地转需的最小净空 | 0.192 m（车体外接半径） | **0.417 > 0.192，能转** |
| 往南 0.42 m 处的缝 S 代价 | `(0.70,−2.00)` cost 0 | **零代价，畅通** |
| 缝 S 宽度 | 0.655 m | 远宽于车宽 0.400 |
| 目标点 `(2.20,−1.50)` 净空 | 0.750 m | 开阔 |

结论：**路是通的，车完全有能力走，它只是不知道该往哪个方向转。**

> ★ 这一步不能跳。如果这一步就否了，后面所有控制器分析都是白费。

---

## 二、为什么车「不动」而不是「转错方向」

先排除运气问题：日志里三种恢复行为**全试过**了。

```
1794309214  behavior_server: Running spin      (转 1.57 rad, 6.2s)
1794309240  behavior_server: Running wait      (5s)
1794309266  behavior_server: Running backup    (7.2s)
```

`spin` 原地转了、`backup` 倒了、`wait` 等了，177 秒后照样 `Goal failed`。
spin 能转（说明底盘能原地转），但转完了 DWB 接手又不动。

所以问题不在底盘、不在恢复行为，**在 DWB 的打分逻辑**。

---

## 三、根因：v=0 时，四个 position critic 全部退化

DWB 每个周期采样 2000 条候选轨迹（`20 vx × 5 vy × 20 vtheta`），
每条被 critic 打分，总分最高者胜出。关键在**五个 critic 各自看什么**：

| critic | scale | 它算的量 | 原地转（v=0, w≠0）时 |
|---|---|---|---|
| `PathAlign` | 32.0 | 轨迹末端 + `fpd`×朝向 的点 → 到路径的横向偏移 | ★ **末端恒在车心**，`fpd`=0.1 时区分度只有 0.2 m |
| `PathDist` | 32.0 | 轨迹末端到路径的距离 | ★ 同上，几乎恒定 |
| `GoalAlign` | 24.0 | 同上，朝目标方向 | ★ 同上 |
| `GoalDist` | 24.0 | 轨迹末端到目标距离 | ★ 同上 |
| `RotateToGoal` | 32.0 | 朝向与目标朝向的差 | ★ **只在距目标 < `xy_goal_tolerance` 时生效** |

最后一行是致命的。`RotateToGoal` 的源码逻辑是：

```cpp
// nav2_dwb_controller/.../rotate_to_goal.cpp
double RotateToGoalCritic::scoreTrajectory(Trajectory2D & traj)
{
  if ((goal_x_ - pose.x_) * (goal_x_ - pose.x_) +
      (goal_y_ - pose.y_) * (goal_y_ - pose.y_) >
      xy_goal_tolerance_ * xy_goal_tolerance_) {
    return 0.0;      // ← 远离目标时直接返回 0，完全不参与打分
  }
  ...
}
```

车距目标 **1.50 m**，而 `xy_goal_tolerance` 是 **0.12 m**。
⇒ `RotateToGoal` 权重 0，**唯一能表达「该往哪转」的 critic 被关掉了**。

于是 DWB 面对「原地转」这件事时：
- 转左（w>0）和转右（w<0）的所有 critic 分数**几乎完全相同**
- 它只能按采样顺序挑一个，下一轮 `Oscillation` critic 发现方向反了
  （`oscillation_reset_dist` 默认 0.05 m，原地转时位移 0 → 必然触发）
  → 该轨迹分数设为无穷大 → 淘汰
- 再挑一个、再被淘汰……**永远在原地抖**

> ★ 这个 bug 的本质：**critic 是「位置型」的评价函数，
>   而原地转产生的是「朝向型」的进展。两者维度不匹配。**
>   调权重治不好——信息量是 0，加权只是放大噪声。

---

## 四、为什么之前调 inflation / 容差都没用

S27 的修法（inflation 0.25→0.22、容差 0.15→0.12）针对的是**缝宽门槛**，
这次东南角走的是**缝 S（0.655 m，零代价通道）**，压根不经过缝 N。

验证：`(0.70,−2.00)` 到 `(1.20,−2.00)` 全段 cost = 0。
⇒ 这个点和膨胀半径毫无关系。**换了个病因，得换药。**

---

## 五、修法：用 RotationShimController 把这个情形拿走

`nav2_rotation_shim_controller` 是官方为「大角度起步」做的**包装器**：

```
车头 vs 路径起始切线夹角 > angular_dist_threshold
        │
        ├── 是 → 先原地转到对准路径（v=0, w=±rotate_to_heading_angular_vel）
        │        转完后把控制权交给内层
        │
        └── 否 → 直接转交内层
```

**为什么这个方案对、而不是继续调 DWB 的 critic：**

| 方案 | 性质 | 问题 |
|---|---|---|
| 调 `PathAlign.scale` / `PathDist.scale` | 改权重 | 原地转时信息量为 0，改权重只是改噪声 |
| 提高 `forward_point_distance` | 改退化的量 | 缓解但治不好（0.1→0.25 只是把区分度从 0.2 提到 0.5） |
| **RotationShim 包一层** | **改结构** | ★ 把这个情形**从 DWB 手里拿走**，交给一个专职做这事的组件 |

我在这一轮里**三个都做了**（见下），但只有第三个是治本的。

### 配置结构

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:                                    # ← 外层：shim
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.35                 # 20° 以上才触发原地转
      rotate_to_heading_angular_vel: 0.8
      max_angular_accel: 2.0
      rotate_to_heading_when_stopped: true         # ★ 差速车必须
      closed_loop: false
      primary_controller: "FollowPathDWB"          # ← 指向内层

    FollowPathDWB:                                 # ← 内层：原 DWB
      plugin: "dwb_core::DWBLocalPlanner"
      ...（原有全部参数不变）
```

**两个名字必须不同。** 如果内层也叫 `FollowPath`，shim 会把自己当内层 → 无限递归。
这是 `check_controller.py` 的第一条检查项。

### 三个关键参数

| 参数 | 值 | 为什么 |
|---|---|---|
| `angular_dist_threshold` | 0.35 rad (20°) | 设 1.57 等于不启用；设 0.05 会频繁打断正常跟踪 |
| `rotate_to_heading_when_stopped` | **true** | 车前方 0.17 m 是墙，必须原地转；false 会「边转边走」直接撞 |
| `closed_loop` | false | 用当前位姿算角度差，慢速车更稳（开环不会因跟随滞后反复触发） |

---

## 六、必须配套改 `progress_checker`，否则白修

这是**最容易漏的一步**，而且症状和"没修好"一模一样。

`SimpleProgressChecker` 只看**位移**，不看转角：

```cpp
// nav2_controller/.../simple_progress_checker.cpp
double curr_pos_x, curr_pos_y;
... 
return hypot(curr_pos_x - base_pos_x, curr_pos_y - base_pos_y) >= movement_radius;
```

而 `RotationShim` 的第一步就是**原地转**——位移恒为 0！

时间账（`rotate_to_heading_angular_vel` 0.8、`max_angular_accel` 2.0、转 70°）：

```
梯形速度曲线：加速 0.40s + 匀速 1.13s + 减速 0.40s = 1.93s
+ 内层 DWB 起步                          1~2s
+ planner 降频抖动（实测有 1.67Hz 时段）   2~3s
────────────────────────────────────────────────
一次「转 + 起步」保守                      5.9s
若再触发一次恢复动作（spin 6s / backup 7s） 12.9s
```

原来的 `movement_time_allowance: 10.0` **不足以覆盖**，会反复踩线，
表现为「刚转完就 Failed to make progress」，然后恢复、重规划、再转……

⇒ 改成 **25.0**。既能容下「转 + 起步 + 一次恢复」，真卡死（顶墙不动）仍会在 25 秒内被抓到。

> ★ 我第一版给 `check_controller.py` 设的下限是 8.0 秒，**故障注入测试立刻打脸**：
> 把 allowance 从 25 改回 10，检查仍然全通 —— 这个判据等于摆设。
> 重新按上面的时间账算出 12.9s，取下限 15s，10 才被正确判 FAIL。

---

## 七、同时做的两处 DWB 调整（治标）

这两处不是主力，但在「角度差小于 20°、shim 不介入」时仍有帮助：

**① `forward_point_distance` 0.1 → 0.25**

`PathAlign` 的计分点是「轨迹末端 + fpd × 朝向」。原地转时这个点只在半径 `fpd` 的圆上移动，
区分度上限就是 `2×fpd`：0.1 时最多差 0.2 m，0.25 时 0.5 m，乘 scale 32 后分差从 6.4 涨到 16。
★ 别超过 0.3，否则直行时计分点会飘到拐角外，导致切弯。

**② 加 `Twirling` critic（scale 0.01）**

唯一「只惩罚原地转、不判断转向对不对」的 critic。
**刻意设小**：它只给「原地打转」轻微扣分，让 DWB 优先考虑「边转边走」——
那种情形下 `PathAlign` 恢复有效，shim 不容易被触发，减少不必要的中断。
设太大会让车宁愿顶墙也不转，更死。

另：`BaseObstacle.scale` 0.02 → 0.1。官方 0.02 是给「0.1m 分辨率 + 大车」的，
本项目 0.05 分辨率下 253 代价 × 0.02 才 5.06 分，而 `PathAlign` 是 32，
差一个数量级 → 车对贴墙几乎无感。

---

## 八、新增检查脚本：check_controller.py

为什么需要它：这类问题**yaml 语法完全合法、ROS 也能正常起**，
只是「控制器之间的关系」错了，运行时才崩，而且报错不指向根因。

```bash
python3 tools/verify/check_controller.py
```

检查 9 项：

| # | 检查 | 抓什么 |
|---|---|---|
| ① | `controller_plugins` 只有一个、有对应参数块 | 配多个会让行为树选错 |
| ② | `primary_controller` 不自指、目标存在、内层是 DWB 且有真配置 | 无限递归 / 找不到内层 / 内层空壳 |
| ③ | `angular_dist_threshold` / `rotate_to_heading_angular_vel` / `when_stopped` | shim 不介入 / 转太慢 / 边转边走撞墙 |
| ④ | `progress_checker.allowance >= 15s` | ★ 原地转位移为 0，被误判没进展 |
| ⑤ | 内层 `min_vel_x` | 提示是否允许倒车 |

**故障注入验证**（4 个都抓到）：

```
注入 primary_controller 指向自己  → FAIL 会无限递归
注入 primary_controller 写错名    → FAIL shim 会报 Failed to create controller
注入 when_stopped: false          → FAIL 差速车窄处会边转边走撞上去
注入 allowance: 10.0              → FAIL 一转向就被判没进展
```

已接进 `live.sh nav` 作为「[0/3] 参数闸门」，起仿真前 2 秒跑完；
不过不会硬中断，而是大声打印 FAIL 并等确认（`y` 继续）。

---

## 九、这次的教训

**① 「能规划」和「能执行」之间还隔着「控制器看得懂」。**
S27 讲的是缝宽门槛（几何），这次是控制器盲区（算法）。三层都要过。

**② 看到「车不动」，先分清三种不动：**

| 现象 | 判据 | 病因层 |
|---|---|---|
| 路径不刷新 + `Failed to create path` | `grep -c "Passing new path"` = 0 | 规划器 |
| 路径一直刷新 + 车不动 + 前方有路 | 路径在刷、位移 0、净空够 | **控制器（本文）** |
| 路径刷新 + 车抖 + 前方是墙 | 路径在刷、`cmd_vel` 有值 | 代价图/膨胀 |

第二和第三种最容易混，**关键区别是「前方到底有没有路」**——
一定要用 `room_geom.clearance()` 算，不能看 RViz 的色块猜。

**③ 改结构 > 调权重。**
critic 权重是加权投票，某个 critic 信息量为 0 时，调它的权重只是改噪声。
`RotationShim` 改的是「谁来处理这个情形」，这才是正解。

**④ 检查脚本必须做故障注入。**
`MIN_ALLOWANCE_FOR_ROTATE = 8.0` 这个值我拍脑袋写的，注入 `allowance=10` 时
检查照样全 PASS。不注入就发现不了——这和 S27 「自测把当前值当期望值」是同一类错误：
**判据本身没被验证过，就等于没有判据。**

---

## 十、改了什么 / 怎么验

| 文件 | 改动 |
|---|---|
| `nav2_params.yaml` | 控制器改双层（shim + DWB）、`allowance` 10→25、`fpd` 0.1→0.25、加 `Twirling`、`BaseObstacle` 0.02→0.1 |
| `tools/verify/check_controller.py` | **新增**，9 项控制器链检查 + 故障注入自证 |
| `tools/verify/sim_fullchain/live.sh` | `nav` 模式加「[0/3] 参数闸门」 |

验证顺序：

```bash
cd ~/navbot_ws

# ① 离线（2 秒）
python3 tools/verify/check_controller.py     # 应 9/9 全通
python3 tools/verify/check_limits.py         # 应 7/7 全通

# ② 上仿真
bash tools/verify/sim_fullchain/live.sh nav  # 会自动跑上面两个检查
python3 tools/verify/sim_fullchain/nav_precheck.py --wait 90
python3 tools/verify/sim_fullchain/nav_patrol.py

# ③ 如果还卡，按这个查
grep -c "Passing new path" live_nav.log      # 有没有路径
grep "Rotating to heading\|shim" live_nav.log # shim 有没有介入
grep "Failed to make progress" live_nav.log   # 还有多少次
```

**预期**：东南角点应该看到 `shim` 先原地转 ~70°，然后 DWB 接手走缝 S。
日志里会出现 `Rotating to heading` 之类的字样（RotationShim 自己的 INFO 日志）。

> ★ 尚未实跑验证。这次改动涉及控制器结构，建议**先单点验证再跑全巡检**：
> `python3 nav_patrol.py --one 2.20 -1.50`
