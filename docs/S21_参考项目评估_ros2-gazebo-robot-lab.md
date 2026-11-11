# 参考项目评估：ros2-gazebo-robot-lab

> 评估对象：`/home/liang/下载/ros2-gazebo-robot-lab-master.zip`
> 解压位置：`/home/liang/WorkBuddy/2026-11-09-20-52-51/refs/ros2-gazebo-robot-lab-master/`
> 评估日期：2026-11-10
> 结论先说：**代码基本不能抄，方法论文档非常值得学。** 借的是"怎么做实验、怎么记录"，不是"怎么写代码"。

---

## 一、先划清界限：两个项目不是同一个技术栈

| 维度 | 参考项目 | 你的 navbot_ws |
|---|---|---|
| ROS 版本 | **ROS 2 Lyrical** | ROS 2 Humble |
| 仿真器 | **Gazebo Sim 10.5.0**（新版 `gz sim`） | **Gazebo Classic 11.10.2** |
| 运行方式 | Docker 容器 + `--gpus all` | 裸机 + X11 |
| 代价地图插件 | `nav2_costmap_2d::*`（Humble 兼容） | 同 |
| 规划器 | SmacPlanner2D | NavfnPlanner |
| 控制器 | RegulatedPurePursuit | DWB |
| 车辆尺寸 | **2.01 × 1.92 m**（方形大车） | **Ø0.20 m**（圆形小车） |
| 场景 | 24 × 20 m | 6 × 5 m |
| TF 帧名 | `vehicle/odom`、`vehicle/base_link` | `odom`、`base_footprint` |
| 速度话题 | `/model/vehicle/cmd_vel` | `/cmd_vel` |
| 雷达 | `gpu_lidar` + ros_gz_bridge | `ray`（CPU）+ gazebo_ros_ray_sensor |

**直接后果：**

- SDF world 文件**绝对不能拷**——`<sensor type="gpu_lidar">` 是 gz sim 的，Classic 11 不认。
- launch 文件不能拷——它是 `ros_gz_sim` 的 `create`/`bridge`，你是 `gazebo_ros` 的 `spawn_entity`。
- 帧名、话题名全不一样，抄参数会导致 TF 断链。
- 它依赖 Docker + NVIDIA 容器工具链；你没有 CUDA，这条路走不通。

---

## 二、值得借鉴的四件事（按性价比排序）

### ★★★ 1. `safety_contract.py` 的"契约分离"设计

参考实现（`workspace/navigation_base/safety_contract.py`，仅 25 行）：

```python
"""ROS-independent scan contract shared by planning and the final velocity guard."""
import math

STOP_RADIUS_M = 1.9
SCAN_FRAME = 'vehicle/lidar'
BASE_FRAME = 'vehicle/base_link'
SCAN_TIMEOUT_S = 1.5

def scan_state(scan):
    hits = [v for v in scan.ranges
            if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
    valid = (scan.header.frame_id == SCAN_FRAME and bool(len(scan.ranges))
             and (bool(hits) or any(v == math.inf for v in scan.ranges)))
    return min(hits, default=math.inf), valid

def blocked_reason(command_age, scan_age, nearest, valid_scan):
    if command_age > 0.5:
        return 'command timeout'
    if scan_age > SCAN_TIMEOUT_S or not valid_scan:
        return 'scan missing or stale'
    if nearest < STOP_RADIUS_M:
        return f'obstacle inside {STOP_RADIUS_M:g} m stop radius'
    return ''
```

**为什么这个设计好，三点：**

1. **阈值只定义一次。** 规划器和最终 guard 共用 `STOP_RADIUS_M`，物理上不可能出现"规划按 0.3m 算、急停按 0.5m 算"这种互相打架的情况。你项目里 `inflation_radius: 0.25` 在两个 costmap 里各写一遍（注释还特意提醒"保持一致"）——这就是该抽出来的信号。

2. **`blocked_reason()` 返回字符串而不是 bool。** 停车的**原因**是一等公民，能直接进日志、进实验证据。你的 `avoid_test.py` 现在只记"与障碍 0.495 m"，但不知道是哪一条规则拦下来的。

3. **纯函数、零 ROS 依赖。** 可以直接 `import` 进单元测试，不用起仿真。这是它能做到 65 项测试的前提。

**`velocity_guard.py` 里还有两个细节值得抄：**

```python
# ① 速度值本身要做有限性检查——NaN/Inf 会静默穿透所有比较
values = [command.linear.x, command.linear.y, command.linear.z,
          command.angular.x, command.angular.y, command.angular.z]
if not all(math.isfinite(v) for v in values):
    reason = 'invalid velocity'

# ② 用 STEADY_TIME 时钟，不受 /clock 和 sim_time 影响
timer = node.create_timer(0.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
```

②这条尤其重要：你在仿真里 `use_sim_time:=true`，如果 guard 用的是仿真时钟，一旦 Gazebo RTF 掉到 1/3，guard 的 20Hz 实际变成 6.7Hz——**急停会变慢**。这是真实的安全隐患。

### ★★★ 2. 实验证据的归档体系

它的目录长这样：

```
docs/experiments/2026-10-04/
├── README.md                    ← 日结：结论 + 表格 + 明确标注"不成立的部分"
├── evidence/
│   ├── memory-trial-summary.json    ← 机器可读的结论
│   ├── memory-trial-live.json       ← 完整过程（11 万行）
│   ├── memory-trial-before.json     ← 运行前快照
│   └── memory-trial-after.json      ← 运行后快照（含停车验证）
├── images/                      ← 截图
└── memory-maps/                 ← 原始 OccupancyGrid 快照
```

`summary.json` 的字段设计（实测样例）：

```json
{
  "mission_id": "ce973082-963d-4834-ae7c-934453a3e430",
  "limits": {"max_steps": 3, "max_distance": 12.0, "max_sim_time": 180.0,
             "max_wall_time": 900.0, "max_failures": 2},
  "steps": [{"step": 1, "pose_before": [...], "pose_after": [...],
             "distance_m": 1.9365, "minimum_scan_m": 2.7787,
             "status": "succeeded", "stopped": true}],
  "totals": {"distance_m": 4.0265, "minimum_scan_m": 2.5150,
             "net_known_area_change_m2": 31.4275},
  "stop_reason": "no_action_fits_remaining_budget",
  "stop_verification": {"stopped": true, "stop_latched": true,
                        "odometry_fresh": true,
                        "velocity": {"linear_x_m_s": 0.0, "angular_z_rad_s": 0.0}}
}
```

**最值钱的不是字段，是 `stop_verification` 里那个 `velocity` 全零的确认。** "我发了停车指令"和"车真的停了"是两件事，它把这两件分开记录。

**你现在的情况正好缺这个**：`verify_avoid.sh` 打印了四项判据，但没有一份结构化的证据文件可以事隔一个月还复查。

### ★★☆ 3. 日结里"主动划界"的写法

它几乎每段结论后面都跟一句"这不能说明什么"：

> - "不是整屋覆盖率，也不是重置起点后的策略对照"
> - "地图配准变化与动作因果收益分开报告，**不以空间分区数冒充整屋覆盖率**"
> - "第二步存在非整数地图配准变化，**已排除出可信收益趋势**"
> - "此前文档中 30/57/93/119/141 等数字是不同开发阶段的历史测试规模，**不能相加**"
> - "固定请求距离不等于实际里程"
> - "回放使用最终地图且旧路径无 yaw，**不能宣称在线完整复现**"

**这套自我否定的纪律，正是你项目 `PROGRESS.md` 现在最缺的。** 你在 11-09 记录里写过"导航误差 0.134 m"，但没写"这是同一起点、同一地图的单次结果，不是统计意义上的精度"。过一个月回头看，很容易把单次当稳定性能。

### ★★☆ 4. 单元测试的写法：绕过构造函数 + Mock 注入

```python
class ExplorerSafetyTests(unittest.TestCase):
    def explorer(self):
        e = Explorer.__new__(Explorer)     # ★ 不调 __init__，不碰 ROS
        e.latest = {'nearest': 4., 'map_version': 'v1'}
        e.health = Mock()
        e.stop_confirmed = Mock(return_value=True)
        e.pose = Mock(return_value=[3., 5., 0.])
        return e

    def test_existing_guard_stop_never_dispatches_goal(self):
        e = self.explorer(); e.latest['nearest'] = 1.881
        client = self.client()
        result = e.execute(client, object(), 100, 'navigation')
        self.assertEqual(result['error_msg'], 'guard_stop')
        client.send_goal_async.assert_not_called()   # ★ 断言"根本没发目标"
```

最后那行 `assert_not_called()` 是关键：它验证的不是"返回了错误"，而是**"危险时根本没有发出指令"**。这两者天差地别。

代价：`Explorer.__new__` 需要手工摆好所有属性，耦合高、易失效。但这种"安全逻辑离线可测"的能力值得付出这个代价。

---

## 三、明确**不要**借鉴的（逐条说明原因）

### ❌ `inflation_radius: 3.3`

它的车 footprint 是 `[[-1.73,-0.53],[-0.32,-0.96],[0.32,-0.96],[0.32,0.96],[-0.32,0.96],[-1.73,0.53]]`，即 x 跨度 2.05m、y 跨度 1.92m。

`3.3 / 1.92 ≈ 1.72` 倍车宽。换算到你 Ø0.20m 的车 = **0.34 m**。

但你的硬约束是：**0.6 m 窄通道，`2 × inflation_radius < 0.6`，所以上限是 0.3 m**。你现在用 0.25 m，已经接近上限且在通道里验证通过。

**这个参数不仅不能抄，还反证了你的取值得当。**

### ❌ `cost_scaling_factor: 1.0`

它是 1.0，你是 5.0。看着差 5 倍，但注意它的 `inflation_radius` 是 3.3——代价要铺满 3.3 米宽的一圈，如果用 5.0 衰减，出了 0.5 米就没代价了，等于白设膨胀半径。

**两个参数是一对，必须一起看。** 你的 0.25/5.0 组合是为了"窄通道里敢贴墙走"，它的 3.3/1.0 是为了"大车远离一切"。目标不同，不可替换。

### ❌ SmacPlanner2D + RegulatedPurePursuit

换规划器和控制器是一次**全套重新调参**：Smac 需要 `motion_model`、`angle_quantization_bins`、`analytic_expansion_*` 等一组参数；RPP 需要 `lookahead_dist`、`regulated_linear_scaling_min_radius` 等。

你现在的 Navfn + DWB 已经跑出**误差 0.092~0.134 m、动态避障四项全 PASS**。**为了换而换，只会把已验证的东西打回未验证状态。** 这是典型的"优化到变差"。

### ❌ Docker + `--gpus all`

它需要 NVIDIA Container Toolkit + 宿主机 `/dev/dri`。你的机器是 Intel Iris Xe 核显、无 CUDA，且本机装 docker 也有额外风险。这条路是死路。

### ❌ `map_update_interval: 2.0`

它是 2.0，你设了 1.0。这条**你已经做过实验**：1.0 让 RViz 里地图响应更快，且这个参数**不参与优化**，只控制 `/map` 发布频率，改它不影响建图质量。保持 1.0。

### ❌ `minimum_travel_distance: 0.2` / `minimum_travel_heading: 0.2`

它用 0.2，你回滚后用 0.3。**这一条你已经用实验证伪过了**：照抄 AGV 的 0.10 导致建图质量暴跌（未知区 2.5%→20.1%），根因是关键帧密度 ×3 让优化负担 ×10，Gazebo RTF 掉到 1/3。

它的 0.2 是在 Docker + 24×20m 大场景 + GTX1650 硬件渲染下取得的值。**你的约束是 CPI（CPU cycles per iteration）不足，不是参数不对。**

---

## 四、可以直接动手的三件事

### ① 抽出 `safety_contract` 式的常量中心

你现在的阈值散落在各处：

| 位置 | 值 | 含义 |
|---|---|---|
| `nav2_params.yaml:346` | `inflation_radius: 0.25` | global costmap |
| `nav2_params.yaml:418` | `inflation_radius: 0.25` | local costmap（注释写"保持一致"） |
| `avoid_test.py` | `0.30` | 避障判据"绕开距离" |
| `verify_avoid.sh` | 激光阈值 | 同上 |

**建议**：新建 `navbot_bringup/config/safety_limits.md`（或一个 Python 模块），把"车宽 0.2m / 通道 0.6m / 膨胀 0.25m / 避障目标 0.30m"的**推导关系**写清楚，yaml 里只留数值并注明"来源：safety_limits"。

不一定要做成代码（跨语言同步有成本），但至少要有一处**推导依据**，而不是四个数字各写各的。

### ② 给 `avoid_test.py` 加"停车确认"

它现在的判据是"绕开 > 0.30m + 仍到达目标"。缺一条：**最终速度是否真的归零**。

```python
# 结尾加：
twist = 订阅 /cmd_vel 读最后一帧
assert abs(twist.linear.x) < 1e-3 and abs(twist.angular.z) < 1e-3
```

这是它 `stop_verification.stop_latched` 的等价物，成本极低。

### ③ 给实验产出加一份结构化 summary

现在 `verify_nav.sh` / `verify_avoid.sh` 只往 stdout 打。建议在收尾时写一个 JSON：

```
tools/verify/sim_fullchain/evidence/20261110_nav_<timestamp>.json
```

字段不用多：

```json
{
  "script": "verify_nav.sh",
  "timestamp": "2026-11-10T13:20:00+08:00",
  "map": "navbot_room.yaml",
  "goal": [2.0, -1.8],
  "result": "SUCCEEDED",
  "final_pose": [1.95, -1.91],
  "error_m": 0.134,
  "nav2_nodes_active": 8,
  "stop_velocity": {"linear_x": 0.0, "angular_z": 0.0},
  "caveat": "单次结果，同一地图同一起点，非统计精度"
}
```

最后那个 `caveat` 字段是灵魂——它强制你写下这份数据的边界。

---

## 五、一句话总结

| 借鉴什么 | 借鉴什么程度 |
|---|---|
| 安全阈值的**单一来源**（contract 模式） | ★★★ 直接抄设计，值换成你的 |
| 实验证据的**目录结构**（experiments/日期/evidence） | ★★★ 直接抄，成本几乎为零 |
| 结论的**主动划界**写法 | ★★★ 抄纪律，这是最难的也是最值的 |
| 安全逻辑的**离线单元测试** | ★★☆ 学思路，不必照抄 `__new__` 手法 |
| Nav2 / SLAM **参数值** | ☆ 一个都别抄，车和场景完全不同 |
| SDF / launch / 话题名 / 帧名 | ✗ 技术栈不兼容，抄了必崩 |
| Docker + GPU 方案 | ✗ 硬件不支持 |

**最重要的一条**：这个项目难得的地方不是它跑通了什么，而是它**诚实地记录了哪里没跑通、哪些数字不能相加、哪些结论还不能下**。

你项目现在缺的正是这个——不是缺功能，是缺"这份数据能说明什么、不能说明什么"的标注。
