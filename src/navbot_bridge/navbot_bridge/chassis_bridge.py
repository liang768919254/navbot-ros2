#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chassis_bridge —— 真机底盘桥节点（S20 作业 · 参考实现）

把「串口协议」接到「ROS2 图」上：
  上行 O 帧（11 字段，50Hz）STM32 → PC  →  /odom + TF + /joint_states
  下行 V 帧（4 字段，20Hz）  PC → STM32  ←  /cmd_vel

三条硬要求（面试题 D-7 的实证依据）：
  R1  读串口放独立线程，绝不放在 spin 线程里
  R2  用「定时器 + 最新值」发布，不用「收到即发」
  R3  三层异常处理：连接层重连 / 数据层静默丢弃+计数 / 时效层暂停发布
"""

import threading
import time
import math
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------------------
# 协议常量（改这里必须同步改 firmware/navbot_protocol.md 与 test_protocol.py）
# ---------------------------------------------------------------------------
ODOM_FIELDS = 11          # 上行帧字段数 ★ 严格相等，多一个少一个都算坏帧
ODOM_HEADER = "O"
CMD_FIELDS = 4            # 下行帧字段数
CMD_HEADER = "V"
BAUD = 115200             # 8N1，无流控

# ---------------------------------------------------------------------------
# 参数默认值（★ 与 urdf/navbot.xacro 的 left/right_wheel_joint origin 一致）
# ---------------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_WHEEL_SEPARATION = 0.16     # L
DEFAULT_WHEEL_RADIUS = 0.0325       # r
DEFAULT_ENCODER_PPR = 2112.0

PUBLISH_HZ = 50.0                   # /odom 发布频率（跟随上行频率）
STALE_TIMEOUT_S = 0.5               # 时效层：超过这么久没新帧 → 判定陈旧
WARN_THROTTLE_S = 5.0               # 连接层/时效层日志节流间隔
MAX_WHEEL_MMS = 1000                # 下行限幅：单轮 ±1.0 m/s（上游手滑的保险丝）

# /joint_states 里的关节名（★ 与 navbot.xacro 的 joint name 一致）
JOINT_LEFT = "left_wheel_joint"
JOINT_RIGHT = "right_wheel_joint"


# ===========================================================================
# 纯函数区：和 无硬件自测/test_protocol.py 的参考实现一一对应
#   放在模块顶层而不是塞进回调 —— 回调只负责编排，算法必须可以单测
# ===========================================================================
def parse_odom_frame(line: str):
    """上行帧解析。返回 dict，坏帧返回 None。

    ★★ 字段数必须严格等于 11：
       少一个 → 帧被截断，不能猜
       多一个 → 协议版本不匹配，不能忽略
       两种都返回 None，由调用方计入 _bad_frames。
    """
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) != ODOM_FIELDS:
        return None
    if parts[0] != ODOM_HEADER:
        return None
    try:
        return {
            "seq": int(parts[1]),
            "enc_l": int(parts[2]),
            "enc_r": int(parts[3]),
            "vel_l": int(parts[4]),
            "vel_r": int(parts[5]),
            "pwm_l": int(parts[6]),
            "pwm_r": int(parts[7]),
            "vbat_mv": int(parts[8]),
            "tick_ms": int(parts[9]),
            "temp_c100": int(parts[10]),
        }
    except ValueError:
        return None


def cmdvel_to_wheel_mms(v, omega, wheel_sep):
    """差速运动学逆解：(v, ω) → 左右轮 (mm/s)

    ★★ 最该背下来的一对公式：
        v_l = v - ω·L/2
        v_r = v + ω·L/2
    ★ round 不是 int()：int() 截断（向零取整），负数 -0.6 → 0 会带入系统性偏差。
    """
    v_l = v - omega * wheel_sep / 2.0
    v_r = v + omega * wheel_sep / 2.0
    return int(round(v_l * 1000)), int(round(v_r * 1000))


def wheel_mms_to_cmdvel(vel_l_mms, vel_r_mms, wheel_sep):
    """差速运动学正解：左右轮 (mm/s) → (v, ω)  ★ 发 /odom.twist 用"""
    v_l = vel_l_mms / 1000.0
    v_r = vel_r_mms / 1000.0
    v = (v_l + v_r) / 2.0
    omega = (v_r - v_l) / wheel_sep
    return v, omega


def enc_delta_to_mms(delta_enc, wheel_radius, encoder_ppr, dt):
    """编码器脉冲增量 → 线速度 mm/s（和固件 TODO-A2 同一个公式）"""
    m_per_pulse = (2.0 * math.pi * wheel_radius) / encoder_ppr
    return delta_enc * m_per_pulse / dt * 1000.0


def enc_to_rad(enc, wheel_radius, encoder_ppr):
    """编码器累计脉冲 → 轮子累计转角（弧度）★ /joint_states.position 用"""
    m_per_pulse = (2.0 * math.pi * wheel_radius) / encoder_ppr
    return enc * m_per_pulse / wheel_radius


class ChassisBridge(Node):
    """底盘桥：O 帧 → /odom + TF + /joint_states；/cmd_vel → V 帧。"""

    def __init__(self):
        super().__init__("chassis_bridge")

        # ---- 参数（名字别改，verify_e2e.sh 按这些名字传） ----
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("wheel_separation", DEFAULT_WHEEL_SEPARATION)
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("encoder_ppr", DEFAULT_ENCODER_PPR)

        self.port = self.get_parameter("port").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.encoder_ppr = self.get_parameter("encoder_ppr").value

        # ---- 线程间共享状态（R2/R3 都要用；★ 一律在 self._lock 里读写） ----
        self._lock = threading.Lock()
        self._ser = None                 # 串口对象，由读线程负责创建/重连（R3-连接层）
        self._latest = None              # 最新一帧解析结果 dict，或 None
        self._latest_recv_t = 0.0        # 最新一帧到达时刻（时效层判据）
        self._stale_warned = False       # 时效层：只警告一次
        self._bad_frames = 0             # 数据层：坏帧计数（只累计，不打印）
        self._good_frames = 0            # 数据层：好帧计数
        self._last_warn_t = 0.0          # 日志节流用
        self._cmd_seq = 0                # 下行 V 帧序号
        self._closing = False            # 退出标志，读线程靠它收尾

        # ---- 里程计积分状态（只被 _on_publish 在 spin 线程里访问，无需加锁） ----
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_enc_l = None          # 上一帧编码器累计值；None = 还没收到过好帧
        self._prev_enc_r = None
        self._prev_pub_t = None          # 上一帧发布时刻（算 dt）
        self._last_seq = None            # 上一帧 seq，用于时效层的「seq 不增长」判据

        # ---- ROS2 接口 ----
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 发布定时器（R2 的载体）
        self.pub_timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_publish)

        # 读串口线程（R1 的载体）
        # ★ daemon=True：主线程退出时它跟着走，别让它把进程卡住
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # ======================================================================
    # R1 · 读串口线程（★ 独立线程，绝不能放进 spin 线程）
    # ======================================================================
    def _read_loop(self):
        """唯一的职责：搬运字节流 → self._on_line()。不解析、不发话题、不 sleep 长等。"""
        while not self._closing:
            # ---- 1) 打开串口（连接层第 1 道关） ----
            try:
                ser = serial.Serial(self.port, BAUD, timeout=0.1)
            except (serial.SerialException, OSError, ValueError) as e:
                # ★ 打不开就等一等再试，绝不抛异常退出（节点要能一直活着等设备插上）
                self._warn_throttled(f"打开串口 {self.port} 失败，将重试: {e}")
                time.sleep(1.0)
                continue

            # ---- 2) 缓冲区设置（★ hz 偏低的第一排查点，协议文档 §六） ----
            try:
                ser.reset_input_buffer()      # 丢掉打开瞬间的残帧，避免第一帧是半个
            except Exception:
                pass

            with self._lock:
                self._ser = ser
            self.get_logger().info(f"串口已打开: {self.port} @ {BAUD}")

            # ---- 3) 逐行读 ----
            try:
                while not self._closing:
                    raw = ser.readline()
                    if not raw:
                        # timeout 到点、这一轮没读满一行 —— 正常现象，继续转
                        continue
                    self._on_line(raw)
            except (serial.SerialException, OSError, TypeError) as e:
                # ★ 拔线、设备消失都抛在这里 —— 不崩节点，落回外层重连
                self._warn_throttled(f"串口读取出错，将重连: {e}")
            finally:
                # ---- 4) 无论怎么退出内层循环，都要把串口关干净 ----
                with self._lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass
                if not self._closing:
                    time.sleep(1.0)          # 重连前退一步，别把 CPU 打满

        # ---- 5) 收尾：_closing 为真时确保串口已关 ----
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _on_line(self, raw: bytes):
        """一行的落点：数据层（坏帧静默丢弃）+ 时效层状态更新。

        ★★ 绝对不在这里 print / warn —— 第 6 组判据是「日志 < 500 行」，
           20% 乱码下 warn 一次就爆。坏帧只准进计数器。
        """
        # ---- 数据层第 1 关：字节 → 字符串 ----
        try:
            text = raw.decode("ascii", errors="ignore")
        except Exception:
            with self._lock:
                self._bad_frames += 1
            return

        # ---- 数据层第 2 关：字符串 → dict（字段数/头/int 全在这里过滤） ----
        frame = parse_odom_frame(text)
        if frame is None:
            with self._lock:
                self._bad_frames += 1
            return

        # ---- 好帧：加锁更新「最新值」三件套 ----
        now = time.monotonic()
        with self._lock:
            self._latest = frame
            self._latest_recv_t = now
            self._stale_warned = False       # ★ 数据回来了，复位时效层警告标志
            self._good_frames += 1

    def _warn_throttled(self, msg: str):
        """连接层与时效层共用的节流告警：WARN_THROTTLE_S 秒内最多输出一条。"""
        now = time.monotonic()
        # ★ 先判在读，再赋值，避免同一时刻两条告警同时通过
        if now - self._last_warn_t < WARN_THROTTLE_S:
            return
        self._last_warn_t = now
        self.get_logger().warn(msg)

    # ======================================================================
    # R2 · 定时器发布（用「最新值」，绝不阻塞在串口上）
    # ======================================================================
    def _on_publish(self):
        """50Hz 定时器：取快照 → 时效判定 → 编码器积分 → 发 /odom + TF + /joint_states。"""
        # ---- 1) 取快照（加锁只为拿三样东西，拿到就立刻放锁） ----
        with self._lock:
            frame = self._latest
            recv_t = self._latest_recv_t
            stale_warned = self._stale_warned

        now = time.monotonic()

        # ---- 2) 时效层：数据陈旧就**暂停发布**，绝不发旧值 ----
        if frame is None or (now - recv_t) > STALE_TIMEOUT_S:
            if not stale_warned:
                with self._lock:
                    self._stale_warned = True     # 只警告一次，不刷屏
                self._warn_throttled(
                    f"超过 {STALE_TIMEOUT_S}s 没有新帧，暂停发布 /odom")
            return

        # ---- 3) 里程计积分（★ 本节点唯一的算法部分） ----
        # 3a. 首帧保护：没有上一帧就无法算增量，只记录不清零，直接返回
        if self._prev_enc_l is None:
            self._prev_enc_l = frame["enc_l"]
            self._prev_enc_r = frame["enc_r"]
            self._prev_pub_t = now
            self._publish_joint_states(frame, 0.0, 0.0)
            return

        dt = now - self._prev_pub_t
        if dt <= 0.0:
            return          # 时间倒流/重复 tick 的极端情况，跳过这一拍
        self._prev_pub_t = now

        # 3b. 编码器增量 → 左右轮位移（米）
        d_enc_l = frame["enc_l"] - self._prev_enc_l
        d_enc_r = frame["enc_r"] - self._prev_enc_r
        self._prev_enc_l = frame["enc_l"]
        self._prev_enc_r = frame["enc_r"]

        m_per_pulse = (2.0 * math.pi * self.wheel_radius) / self.encoder_ppr
        d_l = d_enc_l * m_per_pulse
        d_r = d_enc_r * m_per_pulse

        # 3c. 轮位移 → 车体位移（差速正解，取平均与轮距）
        d_center = (d_l + d_r) / 2.0
        d_yaw = (d_r - d_l) / self.wheel_separation

        # 3d. 积分成位姿（中点法：用半个 yaw 增量更稳，直行时两者等价）
        yaw_mid = self._yaw + d_yaw / 2.0
        self._x += d_center * math.cos(yaw_mid)
        self._y += d_center * math.sin(yaw_mid)
        self._yaw += d_yaw

        # 3e. 反馈速度 → twist（用 MCU 报的 vel_l/vel_r，比差分更平滑）
        v, omega = wheel_mms_to_cmdvel(
            frame["vel_l"], frame["vel_r"], self.wheel_separation)

        # ---- 4) 发 /odom ----
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"                 # ★ 一字不能错
        odom.child_frame_id = "base_footprint"        # ★ 一字不能错
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        # yaw → 四元数（绕 Z 轴）
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        # ---- 5) 发 TF：odom → base_footprint ----
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform(tf)

        # ---- 6) 发 /joint_states ----
        self._publish_joint_states(
            frame,
            wheel_mms_to_rad_s(frame["vel_l"], self.wheel_radius),
            wheel_mms_to_rad_s(frame["vel_r"], self.wheel_radius))

    # ----------------------------------------------------------------------
    # /joint_states 组装（position 用编码器累计值换算，别用 0 糊过去）
    # ----------------------------------------------------------------------
    def _publish_joint_states(self, frame, vel_l_rad_s, vel_r_rad_s):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [JOINT_LEFT, JOINT_RIGHT]
        js.position = [
            enc_to_rad(frame["enc_l"], self.wheel_radius, self.encoder_ppr),
            enc_to_rad(frame["enc_r"], self.wheel_radius, self.encoder_ppr),
        ]
        js.velocity = [vel_l_rad_s, vel_r_rad_s]
        self.joint_pub.publish(js)

    # ======================================================================
    # 下行 · /cmd_vel → 差速逆解 → V 帧
    # ======================================================================
    def _on_cmd_vel(self, msg: Twist):
        """★ 这个回调跑在 spin 线程里：
          不许 sleep、不许重试等待、不许做阻塞式重连。
          （R1 那份实测要教会你的就是这件事）
        """
        # ---- 1) 逆解：(v, ω) → 左右轮 mm/s ----
        vel_l, vel_r = cmdvel_to_wheel_mms(
            msg.linear.x, msg.angular.z, self.wheel_separation)

        # ---- 2) 限幅 ----
        vel_l = max(-MAX_WHEEL_MMS, min(MAX_WHEEL_MMS, vel_l))
        vel_r = max(-MAX_WHEEL_MMS, min(MAX_WHEEL_MMS, vel_r))

        # ---- 3) 组 V 帧 ----
        seq = self._cmd_seq
        self._cmd_seq = (self._cmd_seq + 1) & 0xFFFF
        line = f"V,{seq},{vel_l},{vel_r}\n".encode("ascii")

        # ---- 4) 写串口：失败只记一笔，交给读线程去重连，绝不让异常冒泡 ----
        with self._lock:
            ser = self._ser
        if ser is None:
            return                                # 串口不在（正在重连）→ 静默丢弃
        try:
            ser.write(line)
        except (serial.SerialException, OSError, TypeError) as e:
            self._warn_throttled(f"下发 V 帧失败: {e}")

    # ======================================================================
    # 收尾
    # ======================================================================
    def destroy_node(self):
        # 1) 让读线程退出循环
        self._closing = True
        # 2) 取走并关闭串口（★ 不关的话 Ctrl+C 后 /dev/ttyUSB0 可能被占住）
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        # 3) 等读线程收尾（★ 别让它在我关串口后还在读）
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        # 4) 交给父类
        super().destroy_node()


def wheel_mms_to_rad_s(mms, wheel_radius):
    """轮子线速度 mm/s → 角速度 rad/s   ★ /joint_states.velocity 用"""
    return (mms / 1000.0) / wheel_radius


def main(args=None):
    rclpy.init(args=args)
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
