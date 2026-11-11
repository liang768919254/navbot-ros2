#!/usr/bin/env python3
"""loc_check.py —— 定位健康检查（判据 + 可复用的库）

★★★ 为什么要这个文件（S34，2026-11-10 夜）

起因：AMCL 的 map->odom 跑出了平面
      Translation [-1.455, 0.421, 3.372]，RPY [-93.5, -70.9, 102.4]
      /amcl_pose 冻结、粒子云 0 个，但巡检照样"跑完"，给出一堆错结论。

第一版防护是 shell 里 grep `ros2 run tf2_ros tf2_echo map odom` 的文本：

    TF_OUT=$(timeout 12 ros2 run tf2_ros tf2_echo map odom | head -20)
    TF_Z=$(echo "$TF_OUT" | grep "Translation:" | sed ... | awk -F, '{print $3}')

**这是错的**：tf2_echo 每 1 秒打一帧，12 秒打 9 帧，
`head -20` 抓到 2~3 帧 → `TF_Z` 是多行字符串 "0.054\n-0.000" →
awk 逐行处理输出 "0\n1" → 字符串比较永远不成立 → 判定形同虚设，
却还打印出"-0.000"这种半行内容，看着像在工作。

⇒ 结论：**不要用文本解析 TF**。用 rclpy 直接查，拿到的是 float，没有解析歧义。

判据（三条，缺一不可）
  L1 map->odom 是平面变换：|z| < 0.02 且 |roll| < 2° 且 |pitch| < 2°
  L2 /amcl_pose 没冻结：车在动时位姿必须变
  L3 odom 与 map 的偏差在合理范围：< 1.0 m

用法
  作为命令行：python3 loc_check.py            # 打印报告，坏了返回 1
  作为库：    from loc_check import check_map_odom, LocReport
"""
import math
import sys

# 阈值
Z_TOL = 0.02            # map->odom 的 z 容差（m）
TILT_TOL = 2.0          # roll / pitch 容差（度）
DRIFT_TOL = 1.0         # odom 与 map 的偏差容差（m）


def quat_to_rpy(x, y, z, w):
    """四元数 → (roll, pitch, yaw)，弧度。"""
    # roll (x 轴)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y 轴)，注意夹紧防 nan
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z 轴)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class LocReport:
    def __init__(self):
        self.ok = True
        self.lines = []
        self.z = None
        self.roll_deg = None
        self.pitch_deg = None
        self.reason = ''

    def add(self, ok, msg):
        self.lines.append(('  ' if ok else '!!') + ' ' + msg)
        if not ok:
            self.ok = False
            if not self.reason:
                self.reason = msg
        return ok

    def text(self):
        return '\n'.join(self.lines)


def check_map_odom(timeout=8.0):
    """查 map->odom，返回 LocReport。

    ★ 用 tf2_ros.Buffer 直接查，不走 CLI、不解析文本。
      必须在 rclpy.init() 之后调用。
    """
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, LookupException, \
        ConnectivityException, ExtrapolationException

    rep = LocReport()
    node = rclpy.create_node('loc_check_probe')
    buf = Buffer()
    _listener = TransformListener(buf, node)  # noqa: F841 必须持有引用

    got = None
    # 轮询等 TF 出现（listener 需要几个回调周期才填满 buffer）
    import time as _t
    t0 = _t.time()
    while _t.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            if buf.can_transform('map', 'odom', Time()):
                got = buf.lookup_transform('map', 'odom', Time())
                break
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            pass

    if got is None:
        node.destroy_node()
        rep.add(False, '%.0fs 内取不到 map->odom（AMCL 没起或没注入初值）'
                % timeout)
        return rep

    tr = got.transform.translation
    rt = got.transform.rotation
    roll, pitch, yaw = quat_to_rpy(rt.x, rt.y, rt.z, rt.w)
    rep.z = tr.z
    rep.roll_deg = math.degrees(roll)
    rep.pitch_deg = math.degrees(pitch)

    rep.add(True, 'map->odom  Translation [%.3f, %.3f, %.3f]  '
                  'RPY [%.1f°, %.1f°, %.1f°]'
            % (tr.x, tr.y, tr.z, math.degrees(roll),
               math.degrees(pitch), math.degrees(yaw)))

    # L1：平面变换
    rep.add(abs(tr.z) < Z_TOL,
            '|z| = %.3f m < %.2f  %s'
            % (abs(tr.z), Z_TOL, '平面变换' if abs(tr.z) < Z_TOL
               else '★ z 不为 0，AMCL 状态跑出平面'))
    rep.add(abs(math.degrees(roll)) < TILT_TOL,
            '|roll| = %.1f° < %.0f°  %s'
            % (abs(math.degrees(roll)), TILT_TOL,
               '' if abs(math.degrees(roll)) < TILT_TOL else '★ 绕 x 轴歪了'))
    rep.add(abs(math.degrees(pitch)) < TILT_TOL,
            '|pitch| = %.1f° < %.0f°  %s'
            % (abs(math.degrees(pitch)), TILT_TOL,
               '' if abs(math.degrees(pitch)) < TILT_TOL else '★ 绕 y 轴歪了'))

    node.destroy_node()
    return rep


def main():
    import rclpy
    rclpy.init()
    try:
        rep = check_map_odom()
        print('=' * 60)
        print(' 定位健康检查（map->odom）')
        print('=' * 60)
        print(rep.text())
        print('-' * 60)
        if rep.ok:
            print(' 结论: 定位正常，可以继续')
            return 0
        print(' 结论: 定位不可信 —— %s' % rep.reason)
        print()
        print(' 处理：')
        print('   1. 在你跑 Nav2 的终端按 Ctrl+C')
        print('   2. pkill -f lifecycle_manager; pkill -f nav2_; sleep 2')
        print('   3. bash tools/verify/sim_fullchain/live.sh nav')
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
