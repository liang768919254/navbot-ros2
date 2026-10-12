#!/usr/bin/env python3
"""chassis_bridge 的无硬件离线验收（mock 串口版）。

为什么需要这个文件
------------------
`无硬件自测/verify_e2e.sh` 的第 5/6 组要求一个真实串口对
（`socat` 建 `/tmp/ttyV0 <-> /tmp/ttyV1`）。容器 / 沙箱 / 无 pty 的环境里
这一步建不出来，那两组会被**跳过**。

但「跳过」不等于「验不了」——桥的输入源只是一段字节流。
用 mock 串口喂给同一个节点类，判据可以一条不少地复核。

    verify_e2e.sh      → 验完整链路（有 pty 时跑）
    本文件             → 验桥本身（任何环境都能跑）

两边判据等价，不会出现「跳过 = 默认过」的盲区。

跑法
----
    source /opt/ros/humble/setup.bash
    source <工作空间>/install/setup.bash
    python3 test_bridge_offline.py

★ 本文件只读不写：不改桥的任何代码，只 import 它的类来驱动。
"""

import sys
import threading
import time
import types

# ===========================================================================
# 在任何 import serial 之前，把 serial 换成 mock
# ===========================================================================
STATE = {
    "seq": 0, "enc_l": 0, "enc_r": 0, "t0": time.time(),
    "garbage": False, "stall": False, "writes": [],
}


class FakeSerial:
    """可编程假串口：正常帧 / 乱码注入 / 卡死，三种模式。

    接口刻意与 pyserial.Serial 一致（readline / write / close），
    这样桥的代码一行都不用改。
    """

    def __init__(self, *a, **kw):
        pass

    def readline(self):
        time.sleep(0.02)                       # 模拟 50 Hz 到达节奏
        if STATE["stall"]:
            time.sleep(0.5)
            return b""                         # 超时无数据
        if STATE["garbage"] and STATE["seq"] % 5 == 0:
            STATE["seq"] += 1
            return b"~~~GARBAGE###\x00\xff not a valid frame at all \n"
        STATE["seq"] += 1
        STATE["enc_l"] += 10
        STATE["enc_r"] += 10
        tick = int((time.time() - STATE["t0"]) * 1000) & 0xFFFFFFFF
        line = (f"O,{STATE['seq']},{STATE['enc_l']},{STATE['enc_r']},"
                f"200,200,50,50,11800,{tick},2530\n")
        return line.encode("ascii")

    def write(self, data):
        STATE["writes"].append(data)
        return len(data)

    def close(self):
        pass


_mock = types.ModuleType("serial")
_mock.Serial = FakeSerial
sys.modules["serial"] = _mock

# ===========================================================================
# 现在才能 import 桥
# ===========================================================================
import rclpy                                          # noqa: E402
from geometry_msgs.msg import Twist                   # noqa: E402
from nav_msgs.msg import Odometry                     # noqa: E402
from sensor_msgs.msg import JointState                # noqa: E402

from navbot_bridge.chassis_bridge import ChassisBridge  # noqa: E402

# 底盘物理参数（★ 必须与 navbot.xacro 一致）
L = 0.16          # 轮距
PUBLISH_HZ = 50.0

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  \033[32m✓\033[0m {name}" + (f"  {detail}" if detail else ""))
    else:
        FAIL += 1
        print(f"  \033[31m✗\033[0m {name}" + (f"  {detail}" if detail else ""))


def main():
    # 参数必须在构造节点之前生效（真机上由 --ros-args -p 传入）
    rclpy.init(args=["--ros-args", "-p", "port:=/mock/ttyV1"])
    node = ChassisBridge()

    got = {"odom": [], "js": []}
    node.create_subscription(Odometry, "/odom", lambda m: got["odom"].append(m), 50)
    node.create_subscription(JointState, "/joint_states",
                             lambda m: got["js"].append(m), 50)
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def spin_for(sec, send_cmd=None):
        t0 = time.time()
        while time.time() - t0 < sec:
            if send_cmd is not None:
                m = Twist()
                m.linear.x, m.angular.z = send_cmd
                cmd_pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.02)

    print("\n── A. 正常流：/odom 频率反推数据通路 ───────────────────────")
    spin_for(2.0)
    n = len(got["odom"])
    hz = n / 2.0
    check("桥进程存活且持续发布", n > 0, f"{n} 条")
    check(f"/odom 频率 ≈ {PUBLISH_HZ:.0f} Hz（对应判据 5.1）",
          0.8 * PUBLISH_HZ <= hz <= 1.2 * PUBLISH_HZ, f"{hz:.1f} Hz")

    print("\n── B. 坐标系命名（Nav2 起不来的关键项）─────────────────────")
    o = got["odom"][-1]
    check("header.frame_id == 'odom'（判据 5.2）",
          o.header.frame_id == "odom", repr(o.header.frame_id))
    check("child_frame_id == 'base_footprint'（判据 5.2）",
          o.child_frame_id == "base_footprint", repr(o.child_frame_id))
    check("twist.linear.x ≈ 0.2 m/s（200 mm/s 换算）",
          abs(o.twist.twist.linear.x - 0.2) < 0.01, f"{o.twist.twist.linear.x:.4f}")
    check("twist.angular.z ≈ 0（左右轮同速）",
          abs(o.twist.twist.angular.z) < 1e-6, f"{o.twist.twist.angular.z:.6f}")
    check("位姿在累积（x 单调增）",
          o.pose.pose.position.x > got["odom"][0].pose.pose.position.x,
          f"x={o.pose.pose.position.x:.4f}")
    check("四元数归一化（w≈1）",
          abs(o.pose.pose.orientation.w - 1.0) < 1e-6,
          f"w={o.pose.pose.orientation.w:.6f}")

    print("\n── C. /joint_states（判据 5.4）─────────────────────────────")
    js = got["js"][-1] if got["js"] else None
    check("/joint_states 有数据", js is not None, f"{len(got['js'])} 条")
    if js:
        check("含 left/right_wheel_joint",
              js.name == ["left_wheel_joint", "right_wheel_joint"], str(js.name))
        check("position 有值（RViz 里轮子会转）", len(js.position) == 2,
              f"pos={[round(p, 3) for p in js.position]}")

    print("\n── D. 下行：/cmd_vel → 逆解 → 串口帧 ───────────────────────")
    STATE["writes"].clear()
    spin_for(1.0, send_cmd=(0.3, 0.5))
    vlines = [w for w in STATE["writes"] if w.startswith(b"V,")]
    check("有下行帧写出", len(vlines) > 0, f"{len(vlines)} 帧")
    if vlines:
        parts = vlines[-1].decode().strip().split(",")
        check("下行格式 V,seq,vl,vr（4 字段）", len(parts) == 4, str(parts))
        # v=0.3, ω=0.5, L=0.16 → vl=300-40=260, vr=300+40=340
        check("逆解正确：v=0.3 ω=0.5 → 260/340",
              parts[2] == "260" and parts[3] == "340", f"{parts[2]}/{parts[3]}")

    print("\n── E. 故障注入：坏数据不崩节点（判据 6.x）──────────────────")
    STATE["garbage"] = True
    before = len(got["odom"])
    spin_for(2.0)
    after = len(got["odom"])
    check("注入乱码后节点仍存活", after > before, f"+{after - before} 条")
    check("乱码未污染 /odom（好帧照常发布）",
          (after - before) / 2.0 > 0.8 * PUBLISH_HZ * 0.8,
          f"{(after - before) / 2.0:.1f} Hz")
    STATE["garbage"] = False

    print("\n── F. 时效保护：数据断了要停发（判据 6.x）──────────────────")
    STATE["stall"] = True
    spin_for(1.5)
    before = len(got["odom"])
    spin_for(1.0)
    after = len(got["odom"])
    check("陈旧数据暂停发布（停发后不再增长）", after == before,
          f"停发期间 +{after - before} 条")
    STATE["stall"] = False
    spin_for(1.0)
    check("数据恢复后自动续发", len(got["odom"]) > after,
          f"恢复 +{len(got['odom']) - after} 条")

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n{'=' * 62}")
    print(f"  通过 {PASS} | 失败 {FAIL}")
    print(f"{'=' * 62}")
    if FAIL == 0:
        print("  ★ 桥的功能判据全过。端到端那两组（物理串口）在有 pty 的机器上再跑。")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
