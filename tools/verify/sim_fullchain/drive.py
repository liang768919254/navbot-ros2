#!/usr/bin/env python3
# ============================================================================
# drive.py —— 用 /cmd_vel 按预设路线驱车（给 SLAM 建图用）
# ============================================================================
# 为什么不直接用 teleop_twist_keyboard：
#   它是交互式的，要人按键。自动化建图需要「按脚本走固定路线」，
#   而且位置必须靠 /odom 闭环，不能靠猜时间。
#
# ★★ 怎么运行（2026-11-10 踩坑后写清楚）：
#   ❌ bash drive.py loop        ← 错！bash 会把 Python 当 shell 脚本逐行读，
#                                  报 "行 18: from: 未找到命令" + 语法错误
#   ✅ ./drive.py loop           ← 对（靠第 1 行 shebang + 可执行权限）
#   ✅ python3 drive.py loop     ← 也可以
#
#   ★ 而且必须在【source 过 ROS 环境】的终端里跑，否则 import rclpy 直接失败：
#       source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh
#     下面有自检，环境不对会直接告诉你怎么做，而不是抛一堆 traceback。
#
# 用法（三种模式，代码里就这三个，没有第四个）：
#   ./drive.py loop      # 带转弯的闭环路线，利于回环检测
#   ./drive.py survey    # 更充分地扫：南北两趟 + 绕回，制造闭合
#   ./drive.py home      # 按 /odom 回到原点（= spawn 点），朝向掰回 +x
# ============================================================================
import math
import sys
import time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
except ModuleNotFoundError as e:
    # ★ 环境没 source 过 ROS 的典型症状。这是最高频的误用，单独给出可执行的指引。
    _missing = getattr(e, 'name', '') or str(e)
    sys.stderr.write(
        "\n"
        "!! 导入失败：找不到 '%s'\n"
        "   原因：当前终端没有加载 ROS 2 环境。\n"
        "\n"
        "   先执行这一行，再重跑本脚本：\n"
        "     source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh\n"
        "\n"
        "   或者一条命令搞定（source 与运行写在同一行）：\n"
        "     bash -c 'source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh; \\\n"
        "              python3 /home/liang/navbot_ws/tools/verify/sim_fullchain/drive.py loop'\n"
        "\n"
        "   ★ 注意调用方式：不要用 `bash drive.py`（那是把 Python 当 shell 脚本），\n"
        "     用 `./drive.py` 或 `python3 drive.py`。\n"
        "\n" % _missing
    )
    sys.exit(2)


class Driver(Node):
    def __init__(self):
        super().__init__('navbot_driver')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.cb, qos)
        # /odom 用 best_effort 更稳（Gazebo 插件默认）
        self.create_subscription(
            Odometry, '/odom', self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, msg):
        self.pose = msg.pose.pose

    def wait_pose(self, timeout=15.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def _pump(self, secs):
        """发一次保持指令 + spin，避免在停顿时丢失订阅"""
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        # ★ 容错：如果 rclpy 的 context 已经失效（超时被 SIGTERM、Ctrl+C、
        #   或 wait_pose 失败后 rclpy 已 shutdown），publish 会抛
        #   RCLError: Failed to publish: publisher's context is invalid
        #   这个异常会把「真正的原因」盖掉，看到的全是 traceback。
        #   停车是收尾动作，失败也不该盖住主流程的结论，所以吞掉并提示。
        if not rclpy.ok():
            return
        try:
            self.pub.publish(Twist())
            self._pump(0.4)
            self.pub.publish(Twist())
            self._pump(0.3)
            self.get_logger().info('已停车')
        except Exception as e:
            self.get_logger().warn('停车指令没发出去（context 可能已失效）: %s' % e)

    # -- 走直线：按 /odom 的位移闭环，走到 dist 米停 -------------------------
    def straight(self, dist, speed=0.15, tol=0.02, timeout=90.0):
        """dist 为正前进、为负后退；按 /odom 实测位移闭环。

        负值用于「贴墙后倒车脱困」这类场景（见 route_home）。
        注意 moved 用 hypot 算的是「走了多远」而不是「往哪走」，
        所以速度方向必须自己乘回 sign。
        """
        self.wait_pose(10)
        sign = 1.0 if dist >= 0 else -1.0
        dist = abs(dist)
        x0, y0 = self.pose.position.x, self.pose.position.y
        cmd = Twist()
        cmd.linear.x = sign * speed
        t0 = time.time()
        while time.time() - t0 < timeout:
            moved = math.hypot(self.pose.position.x - x0,
                               self.pose.position.y - y0)
            if moved >= dist - tol:
                break
            # 位置越接近目标，速度按比例降下来，防止冲过头
            remain = dist - moved
            cmd.linear.x = sign * max(0.05, min(speed, 0.6 * remain + 0.05))
            self.pub.publish(cmd)
            self._pump(0.1)
        self.stop()
        moved = math.hypot(self.pose.position.x - x0,
                          self.pose.position.y - y0)
        if moved < dist * 0.7:
            # 实际只走了目标的一大半 → 大概率撞上了东西
            self.get_logger().warn(
                f'!!! 直线 {dist:.2f}m 只走了 {moved:.3f}m，疑似被障碍挡住 !!!')
        else:
            self.get_logger().info(f'直线 {dist:.2f}m 完成，实际 {moved:.3f}m')
        return moved

    # -- 原地转：转到累计 |yaw 变化| 达 angle 弧度 ---------------------------
    def turn(self, angle, omega=0.6, tol=0.004, timeout=40.0):
        """原地转 angle 弧度（正=逆时针），按 /odom 累计 yaw 闭环。

        ★★ 老版本为什么会「每次都少转 1~2°」（2026-11-10 实测）：
           老逻辑是「累计转过 angle-tol 就 break」，tol=0.03 rad ≈ 1.7°，
           然后立刻 stop()，但停车时仍有一帧角速度在途 → 实际停在
           angle - 1.7° 附近，永远差一点。6 次转向就累积 10°，
           闭环路线也回不到原点。
           实测数据（每次都在 -1~-2°）：-89 / +89 / +89 / +89 / +88。
           离线复现见 turn_sim.py：老逻辑单次残差 -0.96°，7 次累计 +4.8°；
           这里 tol 收到 0.004 rad(0.23°) + 刹停后再读一次 → 单次 0.05°，
           7 次累计 0.24°。

        新逻辑：分三段逼近
          ① 粗转：用额定 omega 转到剩余 25%
          ② 精转：降到 0.4 omega，由比例项收尾
          ③ 收尾：转完先发零速让惯性衰减，再读一次实际 yaw 计入累积
        """
        self.wait_pose(10)
        yaw0 = self._yaw(self.pose)
        accum = 0.0
        prev = yaw0
        cmd = Twist()
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur = self._yaw(self.pose)
            accum += self._wrap(cur - prev)
            prev = cur
            remain = abs(angle) - abs(accum)
            if remain <= tol:
                break
            # ★ 严格单调：只要还没到就继续转，不在「快到了」时提前收
            if remain < abs(angle) * 0.25:
                w = omega * 0.4 * max(0.15, remain / (abs(angle) * 0.25))
            else:
                w = omega
            cmd.angular.z = (w if angle > 0 else -w)
            self.pub.publish(cmd)
            self._pump(0.05)
        # 先刹住，吃掉惯性那一帧，再读最终 yaw
        self.stop()
        self._pump(0.25)
        cur = self._yaw(self.pose)
        accum += self._wrap(cur - prev)
        self.get_logger().info(
            f'转向 {math.degrees(angle):+.0f}° 完成，实际 {math.degrees(accum):+.1f}°'
            f'（残差 {math.degrees(angle - accum):+.1f}°）')
        return accum

    @staticmethod
    def _yaw(pose):
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def route_home(d, tol=0.05):
    """按 /odom 回到原点（= spawn 点），并把朝向掰回 +x。

    用途：上一步验证把车开偏了（典型：撞到 partition_short 停在 x≈1.0、
    车头离墙只剩 0.12m），不必重启 Gazebo 就能复位，省一次人工操作。
    只适用于小距离、路径无障碍的场景，不做避障。
    """
    d.wait_pose(10)
    x = d.pose.position.x
    y = d.pose.position.y
    dist = math.hypot(x, y)
    d.get_logger().info(f'当前位置 ({x:.3f}, {y:.3f})，距原点 {dist:.3f} m')
    if dist < tol:
        d.get_logger().info('已在原点附近')
    else:
        # 车头先转向原点，再直行过去
        yaw = d._yaw(d.pose)
        want = math.atan2(-y, -x)
        delta = d._wrap(want - yaw)
        if abs(delta) > 0.05:
            d.turn(delta)
        d.straight(dist)
    # 朝向掰回 +x：AMCL 初值按 yaw=0 给，朝向差太多定位会偏
    yaw2 = d._yaw(d.pose)
    delta2 = d._wrap(0.0 - yaw2)
    if abs(delta2) > 0.05:
        d.turn(delta2)
    x = d.pose.position.x
    y = d.pose.position.y
    d.get_logger().info(
        f'复位完成：({x:.3f}, {y:.3f}) yaw={math.degrees(d._yaw(d.pose)):.1f}°')


# ---- 矩形走廊的几何常量：route_loop / route_survey 共用，改一处两处都变 ----
# 取值依据见 route_loop 的 docstring，用 loop_geom.py --check 可复验净空。
SOUTH_Y = -2.05      # 南侧走廊（距 partition_short 下沿 0.25，距南墙 0.40）
NORTH_Y = 2.05       # 北侧走廊（距 pillar_a 顶 0.40，距北墙 0.40）
EAST_X = 2.50        # 东侧走廊（距东墙内表面 0.45）
WEST_X = -2.50       # 西侧走廊（距西墙内表面 0.45）


def _drive_rectangle(d, reverse=False):
    """从 (0,0) 朝 +x 出发，沿矩形走廊绕一圈，回到 (0,0)。可反向走。

    正向（reverse=False）：南下→东→北→西→南→东→北上回原点
    反向（reverse=True） ：南下→西→北→东→南→西→北上回原点
    两条路径的障碍净空完全对称，都已用 loop_geom.py --check 验过。
    """
    h = abs(SOUTH_Y)                    # 2.05
    hh = NORTH_Y - SOUTH_Y              # 4.10
    w = EAST_X - WEST_X                 # 5.00
    first_x = EAST_X if not reverse else abs(WEST_X)
    sgn = 1 if not reverse else -1      # 第一次转向的符号跟随绕行方向

    d.turn(sgn * -math.pi / 2)          # 朝南
    d.straight(h)                       # 到南侧走廊
    d.turn(sgn * math.pi / 2)           # 朝东（正向）/ 朝西（反向）
    d.straight(first_x)                 # 到东角 / 西角
    d.turn(sgn * math.pi / 2)           # 朝北
    d.straight(hh)                      # 到北侧走廊
    d.turn(sgn * math.pi / 2)           # 朝西（正向）/ 朝东（反向）
    d.straight(w)                       # 到另一侧的角
    d.turn(sgn * math.pi / 2)           # 朝南
    d.straight(hh)                      # 回到南侧走廊
    d.turn(sgn * math.pi / 2)           # 朝东（正向）/ 朝西（反向）
    d.straight(first_x)                 # 回到南侧走廊中点
    d.turn(sgn * math.pi / 2)           # 朝北
    d.straight(h)                       # 北上回原点 (0,0)


def _finish_at_origin(d, tag=''):
    """开环走完后的统一收尾：掰正朝向 + 位置闭环。

    为什么还要位置闭环：
      turn 修好后单次残差只剩 0.05°，但 7~14 次转向仍会累积 0.5°~1°，
      在 5m 长的走廊末端就是几厘米。用 /odom 直接量误差再补，
      比继续抠开环参数划算得多。
    """
    delta = d._wrap(0.0 - d._yaw(d.pose))
    if abs(delta) > 0.02:
        d.turn(delta)
    x = d.pose.position.x
    y = d.pose.position.y
    dist = math.hypot(x, y)
    d.get_logger().info(
        f'{tag}开环收尾位姿 ({x:+.3f}, {y:+.3f}) 距原点 {dist:.3f} m '
        f'朝向 {math.degrees(d._yaw(d.pose)):+.1f}°')
    if dist > 0.05:
        d.get_logger().info(f'残差 {dist:.3f} m 超过 5cm，调用 route_home 收尾闭环')
        route_home(d)
    else:
        d.get_logger().info('残差在 5cm 以内，无需额外修正')


def route_loop(d):
    """绕房间一整圈后【精确回到起点】(0,0)、朝向恢复 +x —— 制造真正的回环。

    房间几何（逐条抄自 navbot_room.world，改世界要同步改这里）：
      外墙厚 0.1，居中在 x=±3.0 / y=±2.5 → 内表面 x=±2.95, y=±2.45
      partition_mid   (-0.5, 0.5) 2.6×0.1  → x∈[-1.8, 0.8],  y≈0.5
      partition_short ( 1.2,-0.9) 0.1×1.8  → y∈[-1.8, 0.0],  x≈1.2
      pillar_a (1.8, 1.5) r=0.15 ；pillar_b (-1.9,-1.6) r=0.15
      movable_box (-1.5, 1.3) 0.3×0.3

    为什么走廊选 y=±2.05 / x=±2.50（都不是随手取的）：
      南侧必须绕过 partition_short，它下沿在 y=-1.8，
      车半径 0.20 → 走 y=-2.05 时净空 0.25 m，而距南墙还有 0.40 m。
      北侧 pillar_a 上沿 y=+1.65 → 走 y=+2.05 净空 0.40 m。
      东西两侧距墙内表面 ±2.95 → 走 ±2.50 净空 0.45 m。
      用 loop_geom.py --check 可复验每一段的净空。

    ★★ 老版本为什么回不到起点（2026-11-10 定案）：
       旧路线是「南 2.0 → 东 2.4 → 北 4.0 → 西 5.0 → 南 2.2」。
       向东只走 2.4、向西却走 5.0，x 方向净差 -2.6 m，最后那段向南
       只能停在 (-2.6, -0.2)。它不是「走偏了」，而是从设计上就没打算
       回原点 —— 理想情况下终点也是 (-2.600, -0.200)，距原点 2.608 m。
       实车实测终点 (-2.616, -0.195)，与理论值只差 1.6 cm，
       说明车走得非常准，问题完全在路线。

    新路线：位置严格回到 (0,0)；朝向经 _finish_at_origin 掰回 +x。
      （逐段位置用 python3 loop_geom.py 打印，别手算。）
    """
    d.get_logger().info('=== 路线开始：矩形闭环，绕外圈一周 ===')
    _drive_rectangle(d, reverse=False)
    d.get_logger().info('=== 路线结束：矩形闭环，位置已回到原点 ===')
    _finish_at_origin(d)


def route_survey(d):
    """覆盖扫：正反各绕一圈同一矩形，把每面墙从两个方向各扫一次。

    为什么这么设计：
      同一段墙被「两个方向各扫一次」，激光点云对同一面的入射角不同，
      建出来的图边缘更实，回环检测也更容易匹配上（经典的「来回扫」策略）。

    ★★ 老版本的毛病（2026-11-10 用 selftest_drive.py 量出来的）：
       老 survey 净位移 (-0.001, +0.688)，距原点 0.688 m，朝向 180°。
       看起来「回到了起点附近」，其实差了 0.7 m 且车头掉了个方向。

    新设计：第 1 圈顺时针、第 2 圈逆时针走同一矩形，两圈后都精确回原点。
    """
    d.get_logger().info('=== 路线开始：第 1 圈（顺时针）===')
    _drive_rectangle(d, reverse=False)
    d.get_logger().info('=== 路线开始：第 2 圈（逆时针，反向走同一矩形）===')
    _drive_rectangle(d, reverse=True)
    d.get_logger().info('=== 路线结束：两圈扫完，回到原点 ===')
    _finish_at_origin(d)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'loop'

    # ★ 先校验参数，再 rclpy.init()。
    #   原来的写法是把它当字典键用，传错了直接抛裸 KeyError —— 看不到可选项，
    #   第一反应会以为脚本坏了。而且那时已经 init 过了，退出时还要走一遍清理。
    routes = {'loop': route_loop, 'survey': route_survey, 'home': route_home}
    if mode not in routes:
        sys.stderr.write(
            "!! 未知路线: %r\n"
            "   可选: %s\n"
            "   例: ./drive.py loop\n" % (mode, ' | '.join(routes))
        )
        sys.exit(2)

    rclpy.init()
    d = Driver()
    try:
        # ★ 先确认 /odom 真的有数，再决定走不走。
        #   拿不到就别硬走 —— 没有 odom 就没有位移闭环，straight() 会盲跑
        #   到 timeout，车可能一路撞墙。
        print('等待 /odom ...', flush=True)
        if not d.wait_pose(20):
            sys.stderr.write(
                '\n!! /odom 20 秒内没有数据\n'
                '   路线未开始。排查顺序：\n'
                '     1) 仿真起来了吗     ros2 topic hz /scan\n'
                '     2) 差速插件发了吗   ros2 topic hz /odom\n'
                '     3) 若两个都空，仿真进程已退出（后台起的会被回收）\n'
                '   用 live.sh 一口气起 Gazebo+SLAM 最省事：\n'
                '     bash /home/liang/navbot_ws/tools/verify/sim_fullchain/live.sh drive\n')
            return
        print('/odom 已就绪，开始走 %s 路线' % mode, flush=True)
        routes[mode](d)
    except KeyboardInterrupt:
        d.get_logger().info('收到 Ctrl+C')
    except ExternalShutdownException:
        # ★ SIGTERM（timeout 命令、被 kill）时 rclpy 会抛这个。
        #   这属于「外部叫停」，不是 bug —— 但它默认打的是一大段 traceback，
        #   看着像脚本崩了，会误导排查。转成一行提示。
        sys.stderr.write('\n!! 收到外部终止信号（SIGTERM/timeout），已中止\n')
    finally:
        try:
            d.stop()
            d.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
