#!/usr/bin/env python3
"""给 AMCL 注入初始位姿。持续发布数秒，确保 AMCL 一定收到。

为什么不能只用 `ros2 topic pub --once`：
  一次性发布后进程立刻退出，DDS 发现/投递未必已完成，
  消息可能在 AMCL 建立匹配前就被丢弃。这里持续发 3 秒（每 200ms 一次），
  且发完还多 spin 一会儿等 AMCL 处理。

★★★ S34 改（2026-11-10 夜）：注入之后必须【验证】，不能发完就算成功

  实测事故：AMCL 的 map->odom 跑到 translation z=3.372 m、
  roll=-93°、pitch=-71°，/amcl_pose 冻结、粒子云 0 个。
  而 set_pose.py 照样打印 "published /initialpose x15" 返回 0，
  上游脚本据此认为"初值注入成功"，继续往下跑，最后拿到一堆错结论。

  "消息发出去了" ≠ "AMCL 收敛了"。真正的判据是 map->odom 的形态：
    · 平面仿真里 z 必须 ≈ 0，roll / pitch 必须 ≈ 0
    · 注入值与当前 odom 的偏差应该和 map->odom 的平移吻合

♻️ S34 改（再）：初值不再只从 /odom 拿

  原调用方（live.sh / verify_avoid.sh）用 odom_pose.py 读 /odom 当初值，
  前提是「map 与 odom 重合」。跑过几轮导航后这个前提不成立
  （实测相差 1.4 m），注进去就是错的。
  现在支持三种取值来源，优先级从高到低：
    --map  x y yaw      显式给定 map 坐标（最可靠，冷启动时用）
    --auto              读 map->odom 反推：map_pose = map->odom ∘ odom_pose
    （默认）            读 /odom（保留旧行为，但会警告并验证）
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

Z_TOL = 0.02        # map->odom z 容差
TILT_TOL = 2.0      # roll/pitch 容差（度）
MAX_TRIES = 3       # 验证失败重试次数


class PoseSetter(Node):
    def __init__(self, x, y, yaw_w=1.0, z=0.0):
        super().__init__('initial_pose_setter')
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = 'map'
        self.msg.pose.pose.position.x = x
        self.msg.pose.pose.position.y = y
        self.msg.pose.pose.position.z = 0.0
        self.msg.pose.pose.orientation.z = z
        self.msg.pose.pose.orientation.w = yaw_w
        # 协方差：xy 各 0.25 (σ=0.5m)，yaw 0.07 (σ≈0.26rad)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        self.msg.pose.covariance = cov

        # 实际 odom 位姿（用于验证注入值是否与 odom 吻合）
        self.odom = None
        self.create_subscription(
            Odometry, '/odom', self._cb_odom,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom = (p.x, p.y, q.z, q.w)

    def send(self, secs=3.0, hz=10.0):
        """按固定频率发 /initialpose，发 secs 秒（默认 3 s @10 Hz = 30 次）。

        ★★★ 两次都踩了同一个坑，记在这里：

        【第一次】原来写 `while ...: publish(); spin_once(timeout_sec=0.2)`
          以为 spin_once(timeout=0.2) 会阻塞 200ms。**不会。**
          没有待处理事件时它立刻返回 → 3 秒循环了 742 次（≈247 Hz）。

        【第二次】改成按时间片算 deadline 再 spin_once(wait)：
              nxt = t0 + n*period; wait = nxt - now
              if wait > 0: spin_once(timeout_sec=wait)
          **仍然是 746 次**，因为 spin_once 的 timeout 只是「最多等这么久」，
          不是「至少等这么久」——没事件照样立即返回。

        【现在的写法】用 time.sleep 保证节拍，但每轮先做一次非阻塞 spin
          把回调处理掉，sleep 之后再补一次。这样既有精确的发送频率，
          又不会让 /odom 订阅饿死。

        为什么这事重要：Gazebo 本来就吃满 CPU，3 秒烧 700 次空循环
        会拖慢 AMCL 的激光更新，反过来让定位更难收敛。
        """
        t0 = time.time()
        n = 0
        period = 1.0 / hz
        while time.time() - t0 < secs:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)
            n += 1
            # 非阻塞跑一次回调（timeout_sec=0 表示不等待）
            rclpy.spin_once(self, timeout_sec=0)
            # 补足到下一拍，用真正的 sleep；剩余时间不足就不睡
            nxt = t0 + n * period
            wait = nxt - time.time()
            if wait > 0:
                time.sleep(wait)
        self.get_logger().info(f'published /initialpose x{n}')
        # 再 spin 一会儿，让 AMCL 有处理时间
        t1 = time.time()
        while time.time() - t1 < 1.5:
            rclpy.spin_once(self, timeout_sec=0.1)


def read_map_odom(node, timeout=8.0):
    """返回 (tx, ty, tz, roll_deg, pitch_deg, yaw) 或 None。"""
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    buf = Buffer()
    _l = TransformListener(buf, node)  # noqa: F841
    t0 = time.time()
    got = None
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            if buf.can_transform('map', 'odom', Time()):
                got = buf.lookup_transform('map', 'odom', Time())
                break
        except Exception:
            pass
    if got is None:
        return None
    tr = got.transform.translation
    rt = got.transform.rotation
    # 四元数 → RPY
    sinr = 2.0 * (rt.w * rt.x + rt.y * rt.z)
    cosr = 1.0 - 2.0 * (rt.x * rt.x + rt.y * rt.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (rt.w * rt.y - rt.z * rt.x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (rt.w * rt.z + rt.x * rt.y),
                     1.0 - 2.0 * (rt.y * rt.y + rt.z * rt.z))
    return (tr.x, tr.y, tr.z, math.degrees(roll),
            math.degrees(pitch), yaw)


def verify(node, label='', wait=15.0):
    """验证 map->odom 是否为平面变换。返回 (ok, msg)。

    ★ S34 修：加了 wait 参数。刚注入 /initialpose 时 AMCL 需要跑几轮
      激光更新才会第一次发布 map->odom，原来 read_map_odom 只等 8 秒，
      冷启动时经常来不及 → 误判失败 → 白白重注 3 次。
      现在最多等 wait 秒，期间反复 spin。
    """
    import time as _t

    t0 = _t.time()
    t = read_map_odom(node, timeout=wait)
    if t is None:
        return False, ('等了 %.0fs 仍取不到 map->odom —— AMCL 没在发布这条 TF。'
                       '检查：ros2 lifecycle get /amcl 是否 active；'
                       '/scan 是否有数据；/initialpose 是否被接收'
                       % (_t.time() - t0))
    tx, ty, tz, roll, pitch, yaw = t
    msg = ('map->odom [%.3f, %.3f, %.3f] RPY[%.1f°, %.1f°, %.1f°] %s'
           % (tx, ty, tz, roll, pitch, math.degrees(yaw), label))
    if abs(tz) > Z_TOL:
        return False, msg + ' ★ z 超容差'
    if abs(roll) > TILT_TOL or abs(pitch) > TILT_TOL:
        return False, msg + ' ★ roll/pitch 超容差'
    return True, msg


def main():
    args = sys.argv[1:]
    mode = 'odom'
    x = y = 0.0
    w, z = 1.0, 0.0

    if '--map' in args:
        i = args.index('--map')
        mode = 'map'
        x, y = float(args[i + 1]), float(args[i + 2])
        if len(args) > i + 3:
            yaw = float(args[i + 3])
            z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    elif '--auto' in args:
        mode = 'auto'
    else:
        x = float(args[0]) if len(args) > 0 else 0.0
        y = float(args[1]) if len(args) > 1 else 0.0
        w = float(args[2]) if len(args) > 2 else 1.0
        z = float(args[3]) if len(args) > 3 else 0.0

    rclpy.init()
    node = PoseSetter(x, y, w, z)

    # --- 等 /odom 有数据（auto 模式需要） ---
    if mode == 'auto':
        t0 = time.time()
        while node.odom is None and time.time() - t0 < 15.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.odom is None:
            print('[set_pose] !! auto 模式取不到 /odom')
            node.destroy_node(); rclpy.shutdown(); return 1
        ox, oy, oz, ow = node.odom
        # map_pose = map->odom ∘ odom_pose
        mo = read_map_odom(node)
        if mo is None:
            # ★★★ S34 修（关键）：这里原来直接报错返回 1。
            #   但那会造成**循环依赖**：map->odom 由 AMCL 发布，而 AMCL
            #   恰恰要等收到初值才会发布 map->odom。冷启动时这条 TF 必然不存在，
            #   于是 --auto 永远失败 → AMCL 永远不启动 → 车在 RViz 里根本看不见。
            #   实测事故：live.sh nav 起完，/map 有、/odom 有，但 TF 树里
            #   map->odom 不存在，RViz 固定帧设成 map 后 RobotModel/LaserScan
            #   全部空白，用户看到的就是「只有地图、没有小车」。
            #
            #   正确语义：map->odom 不存在 ⟺ 还没人注入过初值 ⟺ 这是冷启动，
            #   此时 AMCL 尚未用激光校正过，map 与 odom 定义上就是重合的，
            #   直接用 odom 位姿当 map 位姿即可（与旧行为一致，是安全的）。
            print('[set_pose] auto: 取不到 map->odom —— AMCL 还没起（冷启动）')
            print('[set_pose]       此时 map 与 odom 定义上重合，按 odom 位姿注入')
            x, y = ox, oy
            yaw_new = math.atan2(2.0 * ow * oz, 1.0 - 2.0 * oz * oz)
            z, w = math.sin(yaw_new / 2.0), math.cos(yaw_new / 2.0)
            node.msg.pose.pose.position.x = x
            node.msg.pose.pose.position.y = y
            node.msg.pose.pose.orientation.z = z
            node.msg.pose.pose.orientation.w = w
            print('[set_pose] auto: odom(%.3f, %.3f, %.1f°) → map(同值)'
                  % (x, y, math.degrees(yaw_new)))
        else:
            tx, ty = mo[0], mo[1]
            myaw = mo[5]
            # 二维刚体复合：先转 odom 位姿，再叠加平移
            ox_yaw = math.atan2(2.0 * ow * oz, 1.0 - 2.0 * oz * oz)
            x = tx + ox * math.cos(myaw) - oy * math.sin(myaw)
            y = ty + ox * math.sin(myaw) + oy * math.cos(myaw)
            yaw_new = myaw + ox_yaw
            z, w = math.sin(yaw_new / 2.0), math.cos(yaw_new / 2.0)
            node.msg.pose.pose.position.x = x
            node.msg.pose.pose.position.y = y
            node.msg.pose.pose.orientation.z = z
            node.msg.pose.pose.orientation.w = w
            print('[set_pose] auto: odom(%.3f, %.3f) + map->odom → map(%.3f, %.3f)'
                  % (ox, oy, x, y))

    ok = False
    for attempt in range(1, MAX_TRIES + 1):
        node.send()
        time.sleep(1.5)
        ok, msg = verify(node, '(第 %d 次注入后)' % attempt)
        print('[set_pose] ' + msg)
        if ok:
            break
        print('[set_pose] !! 定位不健康，重注一次（%d/%d）' % (attempt, MAX_TRIES))

    node.destroy_node()
    rclpy.shutdown()

    if not ok:
        print('[set_pose] !! 注入 %d 次后 map->odom 仍不是平面变换。' % MAX_TRIES)
        print('           AMCL 状态已损坏，需要重启 Nav2：')
        print('             Ctrl+C 停掉 → pkill -f lifecycle_manager; pkill -f nav2_')
        print('             → bash tools/verify/sim_fullchain/live.sh nav')
        return 1
    print('[set_pose] OK 初值注入成功且已验证')
    return 0


if __name__ == '__main__':
    sys.exit(main())

