#!/usr/bin/env python3
# ============================================================================
# avoid_test.py —— 动态避障测试
# ============================================================================
# 测什么：
#   在车【已经在动】的时候，往它前方路径正中插一个 0.3 m 见方的箱子。
#   这个箱子在静态地图上【不存在】，只能靠实时激光 / 局部代价图发现。
#   要证明四件事：
#     P1 激光确实看到了它        （最近距离 < 0.55 m）
#     P2 代价图确实标成障碍了    （局部或全局致命格 > 0）
#     P3 车确实绕开了            （与障碍中心最小距离 > 0.28 m）
#     P4 仍然到达原目标          （终点误差 < 0.30 m）
#
#   ★ P3 单独成立不算数 —— 车可能压根没走到箱子那儿。所以 P4 必须在。
#
# ★★★ S34（2026-11-10 夜）：本脚本自己踩了 S33 同一个坑，已修
#
#   原来用 subprocess.Popen 起 `ros2 action send_goal ... --feedback`，
#   靠 proc.poll() 判断导航是否结束。三个问题：
#     ① 一次性 CLI 在重负载下发现 action server 会超时，目标根本没发出去，
#        但脚本会一直等到 180 s 循环结束，然后报"未知"。
#     ② --feedback 让它持续刷屏，stdout 是 PIPE，缓冲区写满后 CLI 会阻塞。
#     ③ 拿不到 GoalStatus，只能 grep 文本。
#   ⇒ 改用常驻 action client（nav_client.py 同一套思路）。
#
#   还有一个隐藏坑：`ros2 topic echo --once` 每次都要重新建节点发现话题，
#   12 秒采样一次、每次 fork 一个 bash + ros2 进程，在 Gazebo 已满载时
#   会拖慢整条链路。改成 rclpy 直接订阅，一次订阅用到底。
# ============================================================================
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

RUN = os.path.dirname(os.path.abspath(__file__))
# ★★★ S34 改（第三次定用例，这次是算过的）
#
#   第一版 GOAL=(2.7,-2.0)：障碍插点距目标只有 0.206 m < 0.27 m，
#     障碍压在目标点上，P3/P4 会同时假通过。作废。
#
#   第二版 GOAL=(-2.50,1.90)（北走廊）：看着合理，实测跑下来
#     车在障碍旁卡了 60 秒，日志 `Collision Ahead - Exiting Spin` ×1、
#     `failed to create plan` ×2，最后 P3 失败。
#     ★ 算完才明白为什么：北走廊净空 0.550，插一个 0.30 m 的箱子后
#        车心可偏移 = 0.550 - 0.160(墙膨胀) = 0.390
#        障碍禁区   = 0.150 + 0.160(障碍膨胀) = 0.310
#        剩余可通行带 = 0.390 - 0.310 = ★ 0.080 m
#     8 厘米的带里要规划出一条连续路径，规划器基本找不到 → 反复失败。
#     不是避障能力不行，是这个用例把可行空间压到近乎为零。
#
#   第三版 GOAL=(2.30,1.70)（东侧竖通道）：净空恒 0.650，最长的可用直道
#        —— 实测 x=2.30 从 y=-1.80 到 y=+1.80，整整 3.6 m 净空都是 0.650。
#        可通行带算法（同上）：
#        车心可偏移 = 0.650 - 0.160 = 0.490
#        障碍禁区   = 0.310
#        剩余可通行带 = ★ 0.180 m（北走廊 0.080 的 2.25 倍）
#      车从原点出发 → 先进东侧通道 → 北上到 (2.30,1.70)。
#      障碍插在车「当前朝向正前方 1.2 m」，所以一定落在它正在走的方向上，
#      不管此刻是刚进通道还是已在通道里，都有足够侧向余量。
GOAL_X = float(os.environ.get('GOAL_X', 2.30))
GOAL_Y = float(os.environ.get('GOAL_Y', 1.70))
INS_DIST = float(os.environ.get('INS_DIST', 1.2))
OBS_HALF = 0.15          # 方箱半宽
OBS_NAME = 'dyn_obstacle'

# 判据阈值（集中放，好改）
P1_RANGE = 0.55          # 激光最近距离要小于这个
# ★ P3_CLEAR 的含义：车心到障碍中心的最小距离下限。
#   物理下限 = 车半宽 0.12 + 箱半宽 0.15 = 0.27（贴上了）
#   Nav2 的膨胀下限 = 箱半宽 0.15 + inflation 0.16 = 0.31
#     ↑ 只要局部代价图正常工作，车心绝不可能进到 0.31 以内，
#       因为那里是致命格（除非它压根没看见障碍 —— 但那 P1/P2 会先挂）
#   取 0.30 作为判据：比物理贴脸 0.27 高，比膨胀理论值 0.31 低一点，
#   留 1 cm 给「车心恰好在膨胀边界上擦过」的数值抖动。
#   原来写 0.28，介于 0.27 和 0.31 之间 —— 既允许了物理贴脸，
#   又低于膨胀禁区，是个两边不靠的值。
P3_CLEAR = 0.30
P4_ERR = 0.30            # 终点误差
# ★ 障碍插点与目标点的最小安全间距。小于它说明「障碍压住了目标」，
#   这局测的是"能不能到障碍里"，不是"能不能绕开障碍"，测试无效。
MIN_GOAL_SEP = 0.70

# 采样间隔：测到障碍后每 2s 记一次轨迹，够密又不会拖垮仿真
SAMPLE_EVERY = 2.0


class Tracker(Node):
    """一次订阅，全程复用。odom / scan / 两张代价图都走这里。"""

    def __init__(self):
        super().__init__('avoid_tracker')
        # odom / scan 是 BEST_EFFORT（Gazebo 默认）
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose = None
        self.yaw = 0.0
        self.trace = []            # (t, x, y)
        self.scan_min = 99.0
        self.lethal_local = 0
        self.lethal_global = 0
        self.amcl_pose = None      # ★ map 坐标系位姿，用来算轨迹/距离
        self.amcl_yaw = 0.0

        self.create_subscription(Odometry, '/odom', self.cb_odom, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, sensor_qos)
        # ★★ S34 新增：轨迹必须用 map 坐标系。
        #   原来只用 /odom，但 /odom 会漂：实测跑完几轮导航后
        #   map 与 odom 相差 1.4 m（AMCL 会持续修正 map->odom）。
        #   用 /odom 算「距障碍多远」「终点误差多少」全是错的坐标系。
        #   判据 P3/P4 因此必须基于 map 坐标系。
        # ★★ QoS 必须匹配！实测 /amcl_pose 是 RELIABLE + TRANSIENT_LOCAL，
        #   而 QoSProfile(depth=10) 默认是 VOLATILE —— 两者不兼容，
        #   订阅端**一条消息都收不到**，但不会报错，表现为"话题明明有数据却等不到"。
        amcl_qos = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.cb_amcl, amcl_qos)
        # 代价图是 latched（TRANSIENT_LOCAL），深度 1
        cost_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self.cb_local, cost_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.cb_global, cost_qos)

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # ★ trace 不再在这里记录（odom 坐标系），改到 cb_amcl

    def cb_amcl(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.amcl_pose = (p.x, p.y)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.trace.append((time.time(), p.x, p.y))
        self.amcl_yaw = yaw

    def cb_scan(self, msg):
        # ★ inf 是"没打到东西"，不是近距离；必须滤掉
        vals = [r for r in msg.ranges if math.isfinite(r) and 0.12 < r < 11.9]
        if vals:
            self.scan_min = min(vals)

    @staticmethod
    def _lethal(grid):
        # ★ OccupancyGrid 里障碍是 100；代价图里 253/254 是致命/内切。
        #   这里数「>= 99」的格，两种都算。
        return sum(1 for v in grid.data if v >= 99)

    def cb_local(self, msg):
        self.lethal_local = self._lethal(msg)

    def cb_global(self, msg):
        self.lethal_global = self._lethal(msg)


# --------------------------------------------------------------- 常驻 action client
class GoalClient:
    """复用 nav_client.py：一个进程跑完一个目标，拿真实 GoalStatus。"""

    def __init__(self):
        env = os.environ.copy()
        self.proc = subprocess.Popen(
            ['bash', '-lc',
             'source %s/env.sh && exec /usr/bin/python3 %s/nav_client.py'
             % (RUN, RUN)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1, env=env)

    def send(self, gx, gy):
        """异步发一个目标（只写 stdin，不阻塞）。"""
        self.proc.stdin.write('%.4f %.4f\n' % (gx, gy))
        self.proc.stdin.flush()

    def poll(self):
        """非阻塞取结果。有结果返回 dict，没有返回 None。

        ★ 用 select 探测可读，不能直接 readline() —— 那会阻塞掉整个跟踪循环，
          导致障碍插入后采样全停。
        """
        import select
        if self.proc.stdout is None:
            return None
        r, _, _ = select.select([self.proc.stdout], [], [], 0)
        if not r:
            return None
        line = self.proc.stdout.readline()
        if not line:
            return {'state': 'NO_SERVER', 't': 0.0, 'detail': 'nav_client 已退出'}
        try:
            return json.loads(line)
        except ValueError:
            return {'state': 'UNKNOWN', 't': 0.0, 'detail': line[:200]}

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()


# --------------------------------------------------------------- 障碍操作
def _spawn_obstacle(x, y, name=OBS_NAME):
    """★ 必须在独立子进程里 service call。

    本进程持有 rclpy context，同进程内再触发 CLI 会报
    "rcl node's context is invalid"（S33 前踩过）。
    """
    sdf = ('<sdf version="1.6"><model name="%s"><static>true</static>'
           '<pose>0 0 0 0 0 0</pose><link name="link">'
           '<collision name="c"><geometry><box><size>0.3 0.3 0.5</size></box>'
           '</geometry></collision>'
           '<visual name="v"><geometry><box><size>0.3 0.3 0.5</size></box>'
           '</geometry><material><ambient>1 0.1 0.1 1</ambient>'
           '<diffuse>1 0.2 0.2 1</diffuse></material></visual>'
           '</link></model></sdf>' % name)
    payload = ("{name: '%s', xml: '%s', initial_pose: {position: {x: %f, y: %f, "
               "z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, "
               "reference_frame: 'world'}" % (name, sdf, x, y))
    r = subprocess.run(
        ['bash', '-lc',
         'source %s/env.sh && timeout 40 ros2 service call /spawn_entity '
         'gazebo_msgs/srv/SpawnEntity "%s"' % (RUN, payload)],
        capture_output=True, text=True, timeout=60)
    return ('success=True' in r.stdout), r.stdout + r.stderr


def _delete_obstacle(name=OBS_NAME):
    """★ 收尾必删。不删的话下一个测试点会撞上它，
    看起来像'导航突然坏了'，其实是我们自己留的垃圾。
    """
    r = subprocess.run(
        ['bash', '-lc',
         'source %s/env.sh && timeout 30 ros2 service call /delete_entity '
         'gazebo_msgs/srv/DeleteEntity "{name: \'%s\'}"' % (RUN, name)],
        capture_output=True, text=True, timeout=45)
    return 'success=True' in r.stdout


# --------------------------------------------------------------- 预热
def wait_topics(node, timeout=30.0):
    """等 odom / amcl / scan / 代价图都真的有数据。

    ★ 必须等 amcl_pose —— 没有它就只能退回用 odom，坐标系是错的。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.pose is not None and node.amcl_pose is not None
                and node.scan_min < 99.0):
            return True
    return node.pose is not None and node.amcl_pose is not None


def check_localization(node, samples=6, gap=1.0):
    """★ 定位健康检查。返回 (ok, msg)。

    ── S34 第一版（错的）────────────────────────────────────────────
    判据是「连续采样 /amcl_pose，位姿必须有变化（车在动就会变）」。
    实测必失败：调用点在**车还没起步**的时候，车静止 → 位姿本来就恒定。

    更深一层的原因在 nav2_params.yaml：
        update_min_d: 0.10    # 移动超过 10cm 才更新一次
        update_min_a: 0.10    # 转动超过 6° 才更新一次
    AMCL 是**事件驱动**的，车不动时它根本不发新的 /amcl_pose。
    所以「/amcl_pose 不动」在这套配置下是**完全正常**的行为，
    拿它当冻结判据，等于要求一辆停着的车跑出里程。

    ── S34 第二版（现在）───────────────────────────────────────────
    判据换成 map->odom 的形态，这才是 AMCL 健康的直接证据：
      · map->odom 必须存在（不存在 = AMCL 没注入过初值）
      · |z| < 0.02、|roll|、|pitch| < 2°（平面差速车不该有这些分量）
      · odom 与 map 的同一时刻位姿不能差出 1 m 以上（漂移）
    这些都是**静态可测**的，不需要车动。

    「/amcl_pose 冻结」这个症状，真正的表现是 map->odom 跑出平面
    （S34 实测 z=3.372、roll=-93°），用上面的判据能直接抓到。
    """
    import math as _m

    # --- 判据 1：map->odom 存在且是平面变换 ---
    t = read_map_odom(node, timeout=10.0)
    if t is None:
        return False, ('取不到 map->odom —— AMCL 没在发布这条 TF。\n'
                       '           检查：ros2 lifecycle get /amcl（是否 active）\n'
                       '                 ros2 topic echo /scan --once（激光有数据吗）\n'
                       '                 ros2 topic echo /amcl_pose --once（有输出吗）\n'
                       '           若 ③b 定位健康检查刚通过、这里却取不到，\n'
                       '           说明 Nav2 在两步之间挂了：grep ERROR nav.log | tail -20')
    tx, ty, tz, roll, pitch, yaw = t
    desc = ('map->odom [%.3f, %.3f, %.3f] RPY[%.1f°, %.1f°, %.1f°]'
            % (tx, ty, tz, roll, pitch, _m.degrees(yaw)))
    if abs(tz) > 0.02:
        return False, desc + ' ★ z 超容差 0.02 —— AMCL 状态跑出平面'
    if abs(roll) > 2.0 or abs(pitch) > 2.0:
        return False, desc + ' ★ roll/pitch 超容差 2° —— AMCL 状态跑出平面'

    # --- 判据 2：odom 与 map 不能差太远（漂移检测）---
    # 注意：这里只比较**同一时刻**的平移，不比较方向。
    # odom 位姿与 amcl 位姿在静止时应该非常接近（差异就是 map->odom 的平移）。
    drift = _m.hypot(tx, ty)
    if drift > 1.0:
        return False, (desc + ' ★ map->odom 平移 %.2f m > 1.0 m —— '
                       'map 与 odom 已严重脱节，轨迹判据不可信' % drift)

    return True, desc + ' → 平面、无漂移'


def read_map_odom(node, timeout=8.0):
    """读 map->odom，返回 (tx, ty, tz, roll_deg, pitch_deg, yaw) 或 None。

    用 rclpy 直接查 TF，不解析 tf2_echo 的文本
    （那个每 1 秒才打一帧，head -20 会抓到多帧拼成多行，判定会静默失效）。
    """
    import math as _m
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, LookupException, \
        ConnectivityException, ExtrapolationException

    buf = Buffer()
    _l = TransformListener(buf, node)  # noqa: F841 必须持有引用
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            if buf.can_transform('map', 'odom', Time()):
                tf = buf.lookup_transform('map', 'odom', Time())
                tr = tf.transform.translation
                rt = tf.transform.rotation
                sinr = 2.0 * (rt.w * rt.x + rt.y * rt.z)
                cosr = 1.0 - 2.0 * (rt.x * rt.x + rt.y * rt.y)
                roll = _m.atan2(sinr, cosr)
                sinp = max(-1.0, min(1.0, 2.0 * (rt.w * rt.y - rt.z * rt.x)))
                pitch = _m.asin(sinp)
                yaw = _m.atan2(2.0 * (rt.w * rt.z + rt.x * rt.y),
                               1.0 - 2.0 * (rt.y * rt.y + rt.z * rt.z))
                return (tr.x, tr.y, tr.z, _m.degrees(roll),
                        _m.degrees(pitch), yaw)
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            pass
    return None


# --------------------------------------------------------------- 主流程
def main():
    rclpy.init()
    node = Tracker()
    print('[avoid] 目标 (%.2f, %.2f)，障碍插在车前方 %.1f m 路径正中'
          % (GOAL_X, GOAL_Y, INS_DIST))

    # ★★★ S34 新增：用例几何前置检查。
    #   这一轮连换两次用例才想明白：避障测试**不是**随便找条走廊就行。
    #   插一个 0.30 m 的箱子后，规划器要能在「障碍膨胀区」和「墙膨胀区」
    #   之间找到一条连续路径，这个带子太窄就会一直 failed to create plan，
    #   最后得出「避障能力不行」的错误结论 —— 实际是用例把空间压没了。
    #
    #   可通行带 = (车心净空 - inflation) - (障碍半宽 + inflation)
    #   北走廊： (0.550-0.16) - (0.15+0.16) = 0.080  ← 实测卡死 60 秒
    #   东侧道： (0.650-0.16) - (0.15+0.16) = 0.180  ← 现在用的
    #   ★ 判据：< 0.10 m 直接作废，别浪费 4 分钟。
    try:
        import room_geom as G
        half = G.clearance(GOAL_X, GOAL_Y)
        band = (half - G.INFLATION) - (OBS_HALF + G.INFLATION)
        print('[avoid] 目标点净空 %.3f，扣除膨胀后可通行带 %+.3f m'
              % (half, band))
        if band < 0.10:
            print('[avoid] !! 作废：可通行带 %.3f m < 0.10 m' % band)
            print('           插障碍后规划器几乎找不到路径，测不出绕行能力。')
            print('           换一条更宽的通道（净空 >= 0.62），或缩小障碍/膨胀。')
            node.destroy_node(); rclpy.shutdown(); return 6
    except ImportError:
        print('[avoid] (room_geom 不可用，跳过几何检查)')

    if not wait_topics(node):
        print('[avoid] !! 等不到 /odom 或 /amcl_pose 或 /scan，退出')
        node.destroy_node(); rclpy.shutdown(); return 2

    # ★ 定位健康检查：宁可当场退出，也不要跑 4 分钟后给一堆错坐标系的数据
    # ★ S34 修：原来这里打印「6 x 1s」并检查 /amcl_pose 有没有变化 —— 那是错的。
    #   AMCL 是事件驱动的（update_min_d=0.10 / update_min_a=0.10），
    #   车静止时根本不发新的 /amcl_pose，「不动」是正常行为不是冻结。
    #   现在改为检查 map->odom 的形态，静态可测。
    print('[avoid] 定位健康检查（查 map->odom 的 z/roll/pitch）...')
    ok_loc, msg_loc = check_localization(node)
    print('[avoid] %s %s' % ('OK' if ok_loc else '!!', msg_loc))
    if not ok_loc:
        print('[avoid] 退出：定位不可信，后续判据都会算错。')
        print('        典型原因：AMCL 的 map->odom 跑出平面（z/roll/pitch 不为 0），')
        print('                  或 map 与 odom 脱节超过 1 m。')
        print('        处理：重启 Nav2 并重新注入初值（bash live.sh nav）。')
        node.destroy_node(); rclpy.shutdown(); return 7

    # 先清掉上一轮可能残留的箱子
    _delete_obstacle()

    client = GoalClient()
    try:
        return _run(node, client)
    finally:
        client.close()
        _delete_obstacle()
        node.destroy_node()
        rclpy.shutdown()


def _run(node, client):
    print('[avoid] 发送目标...')
    client.send(GOAL_X, GOAL_Y)

    # --- 1. 等车起步（用 map 坐标系的位移，且必须相对起点） ---
    t0 = time.time()
    start = None
    while time.time() - t0 < 90:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.amcl_pose is None:
            res = client.poll()
            if res and res['state'] in ('NO_SERVER', 'REJECTED'):
                print('[avoid] !! 目标未送达：%s' % res.get('detail'))
                return 3
            continue
        if start is None:
            start = node.amcl_pose
            print('[avoid] 起点（map）%s' % ('(%.2f, %.2f)' % start))
            continue
        if math.hypot(node.amcl_pose[0] - start[0],
                      node.amcl_pose[1] - start[1]) > 0.4:
            break
    if start is None or node.amcl_pose is None:
        print('[avoid] !! 收不到 /amcl_pose，退出')
        return 2
    if math.hypot(node.amcl_pose[0] - start[0],
                  node.amcl_pose[1] - start[1]) <= 0.4:
        print('[avoid] !! 车没走出 0.4 m（可能原地转/卡住），退出')
        return 4

    # --- 2. 插障碍（★ 用 map 坐标系；spawn 的 reference_frame 是 world，
    #          在仿真里 world 与 map 重合，所以这里可以直接用 map 坐标） ---
    px, py = node.amcl_pose
    oyaw = getattr(node, 'amcl_yaw', node.yaw)
    ox = px + INS_DIST * math.cos(oyaw)
    oy = py + INS_DIST * math.sin(oyaw)
    print('[avoid] 车在 (%.2f,%.2f) yaw=%.0f°，插障碍到 (%.2f,%.2f)'
          % (px, py, math.degrees(oyaw), ox, oy))

    # ★ 先做有效性检查再插 —— 省得跑完 4 分钟才发现这局压根测不出东西
    sep = math.hypot(ox - GOAL_X, oy - GOAL_Y)
    if sep < MIN_GOAL_SEP:
        print('[avoid] !! 作废：障碍距目标点仅 %.2f m < %.2f m' % (sep, MIN_GOAL_SEP))
        print('           障碍压住了目标，测不出绕行能力。')
        print('           换个目标点，或调 INS_DIST，或让车先从别处出发。')
        return 6
    print('[avoid] 障碍距目标 %.2f m（需 > %.2f），本局有效'
          % (sep, MIN_GOAL_SEP))

    ok, out = _spawn_obstacle(ox, oy)
    print('[avoid] 障碍插入 %s' % ('成功' if ok else '失败'))
    if not ok:
        print(out[-400:])
        return 5

    # ★★★ S34 修：记录插入时刻，P3 只能统计**插入之后**的轨迹。
    #   原来 mind = min(全轨迹到障碍的距离)。但障碍是插在「车前方 1.2 m」
    #   算出来的，也就是**车已经走过的路线上**——插入前车必然路过那个点，
    #   于是 mind 恒约等于 0，P3 在结构上就不可能通过。
    #   实测：P3 报「与障碍最小 0.028 m」，而 Gazebo 里根本没碰撞，
    #   这 0.028 就是插入前车压过该点留下的痕迹。
    t_insert = time.time()

    # 插入后把扫描最低值清零重新开始统计，
    # 否则会拿"插入前正好扫到墙"的读数当成"看到了障碍"
    node.scan_min = 99.0
    time.sleep(2.0)

    # --- 3. 跟踪 ---
    print('[avoid] 跟踪（每 %.0fs 采样一次）...' % SAMPLE_EVERY)
    lethal_l_max = 0
    lethal_g_max = 0
    laser_min = 99.0
    t_start = time.time()
    next_sample = 0.0
    result = None

    while time.time() - t_start < 240:
        rclpy.spin_once(node, timeout_sec=0.1)

        res = client.poll()
        if res is not None:
            result = res
            break

        el = time.time() - t_start
        if el >= next_sample:
            next_sample = el + SAMPLE_EVERY
            laser_min = min(laser_min, node.scan_min)
            lethal_l_max = max(lethal_l_max, node.lethal_local)
            lethal_g_max = max(lethal_g_max, node.lethal_global)
            p = node.amcl_pose or (0.0, 0.0)
            d_obs = math.hypot(p[0] - ox, p[1] - oy)
            print('  [%5.1fs] pos=(%5.2f,%5.2f) 距障碍=%4.2fm '
                  '激光最近=%4.2fm 致命格 L/G=%d/%d'
                  % (el, p[0], p[1], d_obs, node.scan_min,
                     node.lethal_local, node.lethal_global))
            node.scan_min = 99.0      # 每个采样窗口重新累计

    # 超时也是有效结果——说明"没能在 240s 内完成"，不能当成"未知"
    if result is None:
        result = {'state': 'TIMEOUT', 't': 240.0, 'detail': '240s 未结束'}

    # --- 4. 判定 ---
    print()
    print('[avoid] 导航结果: %s  (%.1fs)  %s'
          % (result['state'], result['t'], result.get('detail', '')))

    if not node.trace:
        print('[avoid] !! 没有轨迹，无法判定')
        return 2

    # ★★★ S34 修：P3 只统计「障碍插入之后」的轨迹点。
    #   原来 min 取全轨迹。但障碍插在车已走过的路线上（车前方 1.2 m 处），
    #   插入前必然路过该点 → mind ≈ 0 → P3 结构性永不可能通过。
    #   实测报 0.028 m，而 Gazebo 里没有碰撞，正是这个原因。
    after = [(t, x, y) for (t, x, y) in node.trace if t >= t_insert]
    if not after:
        print('[avoid] !! 插入障碍后没有轨迹点，无法判定 P3')
        return 2
    mind = min(math.hypot(x - ox, y - oy) for _, x, y in after)
    end = node.trace[-1]
    err = math.hypot(end[1] - GOAL_X, end[2] - GOAL_Y)

    # 轨迹剖面：把「距障碍距离」的最小几帧打出来，方便判断是
    # 「缓慢贴边通过」还是「直线穿过」还是「卡住不动」
    prof = sorted(after, key=lambda r: math.hypot(r[1] - ox, r[2] - oy))[:5]
    print('[avoid] 最接近障碍的 5 帧（插入后）：')
    for t, x, y in prof:
        print('   +%5.1fs  (%5.2f,%5.2f)  距障碍 %.3f m'
              % (t - t_insert, x, y, math.hypot(x - ox, y - oy)))

    p1 = laser_min < P1_RANGE
    p2 = (lethal_l_max > 0) or (lethal_g_max > 0)
    p3 = mind > P3_CLEAR
    p4 = (result['state'] == 'SUCCEEDED') and (err < P4_ERR)

    print('=' * 64)
    print('  P1 激光看到障碍      最近 %.3f m  < %.2f   %s'
          % (laser_min, P1_RANGE, 'PASS' if p1 else 'FAIL'))
    print('  P2 代价图标成障碍    致命格 L/G 峰值 %d/%d       %s'
          % (lethal_l_max, lethal_g_max, 'PASS' if p2 else 'FAIL'))
    print('  P3 车确实绕开        与障碍最小 %.3f m  > %.2f   %s'
          % (mind, P3_CLEAR, 'PASS' if p3 else 'FAIL'))
    print('  P4 仍到达原目标      %s  误差 %.3f m  < %.2f    %s'
          % (result['state'], err, P4_ERR, 'PASS' if p4 else 'FAIL'))
    print('-' * 64)

    allok = p1 and p2 and p3 and p4
    # ★ 分情况给结论，不要只说 PASS/FAIL —— 不同组合指向不同病因
    if allok:
        print('  => 避障全流程 PASS：看到了、标记了、绕开了、到达了')
    elif not p1:
        print('  => FAIL：激光压根没看到 ← 查 scan 量程 / 障碍高度 / 插入位置')
    elif not p2:
        print('  => FAIL：看到了但代价图没标 ← 查 obstacle_layer marking / 话题')
    elif not p3 and p4:
        print('  => FAIL：标记了也到了，但贴着走 ← 查 BaseObstacle / inflation')
    elif not p3 and not p4:
        print('  => FAIL：没绕开也没到 ← 查全局重规划 / 局部避障是否生效')
    else:
        print('  => FAIL：绕开了但没到目标 ← 查恢复行为 / 目标点可达性')
    print('=' * 64)
    return 0 if allok else 1


if __name__ == '__main__':
    sys.exit(main())
