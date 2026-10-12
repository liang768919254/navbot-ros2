```python
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
```

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chassis_bridge —— 真机底盘桥节点（S20 作业骨架，★ 核心逻辑留空，你写）

==============================================================================
一、这个文件是干什么的
==============================================================================

把「串口协议」接到「ROS2 图」上。协议本身已经定死了，你**不需要重新发明**：

  协议契约：firmware/navbot_protocol.md（v1）
    ↳ 上行 O 帧（11 字段，50Hz）：STM32 → PC，底盘状态
    ↳ 下行 V 帧（4 字段，20Hz）：PC → STM32，左右轮目标速度
    ↳ 帧格式（一字不能错）：
        O,<seq>,<enc_l>,<enc_r>,<vel_l>,<vel_r>,<pwm_l>,<pwm_r>,<vbat_mv>,<tick_ms>,<temp_c100>\\n
        V,<seq>,<vel_l>,<vel_r>\\n

  参考实现（★「标准答案」在这，但它是**纯函数**）：
    无硬件自测/test_protocol.py
      parse_odom_frame(line)                                → dict | None
      cmdvel_to_wheel_mms(v, omega, wheel_sep)              → (vel_l_mms, vel_r_mms)
      wheel_mms_to_cmdvel(vel_l_mms, vel_r_mms, wheel_sep)  → (v, omega)
      enc_delta_to_mms(delta_enc, wheel_radius, encoder_ppr, dt) → mm/s

  ★ 所以本文件真正要你写的，不是「怎么解析」，而是
    「**怎么把解析接到 rclpy 的线程模型上**」——这才是 S20 的考点。

==============================================================================
二、★ 三条硬要求（面试题 D-7 的实证依据，一条都不能省）
==============================================================================

R1 ★★ 读串口放在**独立线程**，绝不放在 spin 线程里。
   实证：踩坑实测/01_阻塞回调实测_单线程执行器.md
     单线程执行器里，定时器第 3 个 tick 的回调 sleep 6 秒
      → 12 个周期全被吃掉（tick #3 → 6 秒空白 → tick #4）
      → `ros2 service call /ping` 卡 5.124270576 秒
      → Ctrl+C 退不掉
   ★ 你如果把 ser.readline() 写在定时器回调里，就是把这份实测重演一遍。

R2 ★★ 用「**定时器 + 最新值**」发布，不用「收到即发」。
   读线程只管把最新一帧写进共享变量；发布定时器只管按固定周期取走发出去。
   好处：发布节奏与接收节奏解耦，下游拿到的是**等间隔流**，
         /odom 才能稳定在 50Hz（第 5 组的判据是 40~60Hz）。
   ★ 共享变量必须加锁 —— 读线程写、定时器读，是两个线程。

R3 ★★ 三层异常处理，一层都不能省（协议文档 §五）。
   连接层（打不开/拔线）：重连，不崩节点，warn **节流**（5 秒最多一条）
   数据层（字段数≠11 / 头不是 'O' / int() 失败）：**静默丢弃 + 计数**
       ★ 第 6 组的判据：注入 20% 乱码后，日志总行数必须 < 500
         → 坏帧不许 print、不许 warn，只能攒在计数器里
   时效层（seq 长时间不增长 = 数据陈旧）：**暂停发布 /odom**，只警告一次
       ★★ 为什么必须「暂停发布」而不是「继续发旧值」：
          Nav2 的代价地图和 AMCL 会把旧位姿当真值用
          → 车明明停了，代价地图还在滚动 → 定位漂移、规划错乱
          → 而且现象**很隐蔽**，你不会第一时间想到是桥的问题。

==============================================================================
三、验收判据（无硬件自测/verify_e2e.sh，跑完应当是 通过 20+ / 失败 0 / 跳过 0）
==============================================================================

 第 4 组：`ros2 pkg executables navbot_bridge` 里能找到 `chassis_bridge`
 第 5 组：
   □ /odom 话题存在，频率 **40~60 Hz**
   □ /odom 的 header.frame_id == "odom"
   □ /odom 的 child_frame_id == "base_footprint"      ★ 字符串一字不能错
   □ TF 变换 odom → base_footprint 存在
   □ /joint_states 有数据（否则 RViz 里轮子不转）
 第 6 组：
   □ 注入 20% 乱码后进程存活
   □ 注入后 /odom 仍在发布
   □ 日志总行数 < 500（不刷屏）

==============================================================================
四、参数（★ 已和 verify_e2e.sh:395 的传参对齐，别改名）
==============================================================================

  port              串口设备   真机 /dev/ttyUSB0 ; 无硬件自测 /tmp/ttyV1
  wheel_separation  轮距 L=0.16 m   ★ 必须与 urdf/navbot.xacro 一致
  wheel_radius      轮半径 r=0.0325 m ★ 必须与 urdf/navbot.xacro 一致
  encoder_ppr       编码器每转脉冲 2112.0
                    ★ 自检换算：2112/(2π×0.0325) = 10342.6 脉冲/米
                      ≈ 踩坑实测/02 里那个锚点「车走 1 米 = 10343 个脉冲」

==============================================================================
"""

import threading
import time
import math
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
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

# /joint_states 里的关节名（★ 与 navbot.xacro 的 joint name 一致）
JOINT_LEFT = "left_wheel_joint"
JOINT_RIGHT = "right_wheel_joint"


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

        # ---- ROS2 接口（★ 建议自己动手敲一遍，这是 ROS2 节点最标准的开胃菜） ----
        # TODO(1) 建 /odom 发布器
        #   nav_msgs/msg/Odometry，话题 "/odom"，队列长度 10
        #   提示：self.create_publisher(...)  → 存成 self._odom_pub
        self._odom_pub = self.create_publisher(Odometry,"/odom",10)
        # TODO(2) 建 /cmd_vel 订阅器
        #   geometry_msgs/msg/Twist，话题 "/cmd_vel"，回调 self._on_cmd_vel
        self._cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self._on_cmd_vel,10)
        # TODO(3) 建 /joint_states 发布器
        #   sensor_msgs/msg/JointState，话题 "/joint_states"，队列长度 10
        self.joint_pub = self.create_publisher(JointState,"/joint_states",10)

        # TODO(4) 建 TF 广播器
        #   self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        # TODO(5) 建发布定时器（R2 的载体）
        #   self.create_timer(1.0 / PUBLISH_HZ, self._on_publish)
        self.pub_timer = self.create_timer(1.0 / PUBLISH_HZ,self._on_publish)
        # TODO(6) 启动读串口线程（R1 的载体）
        #   threading.Thread(target=self._read_loop, daemon=True).start()
        #   ★ daemon=True：主线程退出时它跟着走，别让它把进程卡住
       
        self._thread = threading.Thread(target=self._read_loop,daemon=True)
        self._thread.start()
    # ======================================================================
    # R1 · 读串口线程（★ 独立线程，绝不能放进 spin 线程）
    # ======================================================================
    def _read_loop(self):
        """要求（按这个顺序做，缺一条都会在第 6 组翻车）：

        1. 循环直到 self._closing 置位
        2. 打开串口（用 self.port 与 BAUD）
             ★ 打不开 → 走连接层：warn 节流 + sleep 后重试，**绝不抛异常退出**
             提示：import serial（pyserial，e2e 第 0 组已确认系统 python 有）
        3. ★ 缓冲区设置：串口默认可能带缓冲，O 帧会被攒着延迟到达。
             你要设成**行缓冲/无缓冲**，否则 /odom 的频率会莫名其妙偏低。
             这一条是「hz 明显低于 50」时的第一排查点（协议文档 §六）。
        4. 逐行读：读到一行 → self._on_line(line)
             ★ 不要在这里做解析、更不要在这里发话题 —— 这里只负责「搬运」
        5. 任何读异常（拔线会抛）→ 关掉串口、走连接层重连
        6. 收尾：self._closing 为真时关掉串口再 return
        """
        # TODO(R1) 实现
        # --- 1a：先只验「打不开时不崩」---
        try:
            self._ser = serial.Serial(self.port, BAUD, timeout=0.2)
        except serial.SerialException as e:
            self._warn_throttled(f"打开串口失败: {e}")
            time.sleep(1.0)
            return
        while not self._closing:
            line = self._ser.readline()
            if not line:
                continue
            self._on_line(line)

        # while not self._closing:
        #     line = self._ser.readline()
        #     raise NotImplementedError("R1 未实现：读串口线程")
        # def parse_odom_frame(line):
        #     """
        #     上行帧解析（这是你将来要在 chassis_bridge 里写的那份）。
        #     返回 dict，坏帧返回 None。

        #     ★★ 关键设计：字段数必须严格等于 11。
        #     少一个 → 说明帧被截断了，不能猜
        #     多一个 → 说明协议版本不匹配，不能忽略
        #     两种情况都返回 None，由调用方计入坏帧数。
        #     """
        #     line = line.strip()
        #     if not line:
        #         return None
        #     parts = line.split(",")
        #     if len(parts) != 11:
        #         return None
        #     if parts[0] != "O":
        #         return None
        #     try:
        #         return {
        #             "seq": int(parts[1]),
        #             "enc_l": int(parts[2]),
        #             "enc_r": int(parts[3]),
        #             "vel_l": int(parts[4]),
        #             "vel_r": int(parts[5]),
        #             "pwm_l": int(parts[6]),
        #             "pwm_r": int(parts[7]),
        #             "vbat_mv": int(parts[8]),
        #             "tick_ms": int(parts[9]),
        #             "temp_c100": int(parts[10]),
        #         }
        #     except ValueError:
        #         return None

    def _on_line(self, raw: bytes):
        """一行的落点：数据层 + 时效层（★ 这里是最容易写错的地方）。

        要求：
        1. decode 成字符串；解码失败 → self._bad_frames += 1，return
        2. 调 parse_odom_frame（可自己照 test_protocol.py:44 抄一份，
           也可以 import 过来）；返回 None → self._bad_frames += 1，return
        3. 成功 → 在 self._lock 里更新三样：
             self._latest         = 解析出来的 dict
             self._latest_recv_t  = time.monotonic()
             self._stale_warned   = False      ★ 数据回来了，把时效层的警告标志复位
           self._good_frames += 1

        ★★ 绝对不要在这个函数里 print 或 self.get_logger().warn
           —— 第 6 组判据是「日志 < 500 行」，20% 乱码下 warn 一次就爆
           —— 坏帧只准进计数器，要报也只准在定时器里按秒汇总报一行
        """
        # TODO(R3-数据层) 实现
        raise NotImplementedError("R3 数据层未实现：坏帧静默丢弃 + 计数")

    def _warn_throttled(self, msg: str):
        """连接层与时效层共用的节流告警：WARN_THROTTLE_S 秒内最多输出一条。

        提示：self._last_warn_t + self.get_clock().now() 或 time.monotonic()
        """
        # TODO(R3-连接层) 实现
        raise NotImplementedError("R3 连接层未实现：节流告警")

    # ======================================================================
    # R2 · 定时器发布（用「最新值」，绝不阻塞在串口上）
    # ======================================================================
    def _on_publish(self):
        """要求（6 步，第 3 步是真正的技术活）：

        1. 取快照：在 self._lock 里读出 self._latest / self._latest_recv_t
        2. 时效层判定：
             快照为 None，或 now - recv_t > STALE_TIMEOUT_S
               → ★★ 直接 return，**暂停发布 /odom**（不要发旧值！理由见 R3）
               → 若 self._stale_warned 还是 False，打一条节流 warn 并置 True
                  （这样连丢 10 秒也只说一句）
        3. 算里程计（★ 本节点唯一的算法部分，也是面试要问的地方）：
             用 enc_l / enc_r 的**增量** + wheel_radius + encoder_ppr + dt
             参考无硬件自测/test_protocol.py:121 enc_delta_to_mms()
             再积分成位姿（x, y, yaw）
             ★ 三条路你选哪条，理由是什么，写进 docs/ 的 S20 复盘：
                 (a) 用编码器增量积分（推荐：最接近真实里程计）
                 (b) 用 vel_l/vel_r 反馈 × dt 积分
                 (c) 直接把 vel 当瞬时速度、位姿不做积分
             ★ 注意：这是「估计」，一定会漂移 —— 漂移是靠 /scan + AMCL 修的，
               不是你在这里能解决的。你的任务是把漂移控制在合理范围。
        4. 发 /odom：
             header.stamp  = now
             header.frame_id     = "odom"            ★ 一字不能错
             child_frame_id      = "base_footprint"  ★ 一字不能错
             twist.twist.linear.x / angular.z = 正解结果
               （用 test_protocol.py:106 wheel_mms_to_cmdvel）
        5. 发 TF：odom → base_footprint，用同一个位姿
             提示：TransformBroadcaster.sendTransform(TransformStamped(...))
        6. 发 /joint_states：
             name     = [JOINT_LEFT, JOINT_RIGHT]
             position = 两轮的转角（弧度）★ 用编码器累计值换算，别用 0 糊过去
             velocity = 两轮的角速度
        """
        # TODO(R2) 实现
        def enc_delta_to_mms(delta_enc, wheel_radius, encoder_ppr, dt):
            """编码器脉冲增量 → 线速度 mm/s（和固件 TODO-A2 同一个公式）"""
            m_per_pulse = (2.0 * math.pi * wheel_radius) / encoder_ppr
            return delta_enc * m_per_pulse / dt * 1000.0
        raise NotImplementedError("R2 未实现：定时器 + 最新值发布")

    # ======================================================================
    # 下行 · /cmd_vel → 差速逆解 → V 帧
    # ======================================================================
    def _on_cmd_vel(self, msg: Twist):
        """要求：

        1. 逆解：(v, ω) → 左右轮 mm/s
             用 test_protocol.py:91 cmdvel_to_wheel_mms(msg.linear.x, msg.angular.z, self.wheel_separation)
             ★ 公式：v_l = v - ωL/2，v_r = v + ωL/2
             ★ L 用 self.wheel_separation（参数），不许写死 0.16
        2. 限幅：给个速度上限（自己定，比如 ±1.0 m/s），防止上游手滑发个 10 m/s 把车送走
        3. 组 V 帧：f"V,{self._cmd_seq},{vel_l},{vel_r}\\n"，然后 self._cmd_seq += 1
        4. 写串口
             ★ 写失败（串口没了）→ 交给连接层重连，别让异常冒泡崩掉节点
        5. ★ 这个回调跑在 spin 线程里：
             **不许 sleep、不许重试等待、不许做串口重连的阻塞动作**
             （这正是 R1 那份实测要教会你的事）

        ★ 顺带想一下：这条和 R2 的第 2 步是同一枚硬币的两面 ——
          上行没数据时你「停发 /odom」，那下行该不该也「停发 V 帧」？
          （提示：MCU 侧有 500ms 失联自动停车兜底，见协议文档 §四）
          你更该关心的是：你的节点崩了以后，车会不会停。
        """
        # TODO 实现
        def cmdvel_to_wheel_mms(v, omega, wheel_sep):
            """
            差速运动学逆解：/cmd_vel 的 (v, ω) → 左右轮速度 (mm/s)

            ★★ 这两个公式是整个导航系统里最该背下来的一对：
                v_l = v - ω·L/2
                v_r = v + ω·L/2
            ★ 注意 round 不是 int()：int() 是截断（向零取整），
            对负数会 -0.6 → 0，带来系统性偏差。必须用 round。
            """
            v_l = v - omega * wheel_sep / 2.0
            v_r = v + omega * wheel_sep / 2.0
            return int(round(v_l * 1000)), int(round(v_r * 1000))

        raise NotImplementedError("下行未实现：/cmd_vel → 逆解 → V 帧")

    # ======================================================================
    # 收尾
    # ======================================================================
    def destroy_node(self):
        """要求：
        1. self._closing = True（读线程靠它退出循环）
        2. 关闭串口（★★ 别忘了 —— 不关的话 Ctrl+C 后 /dev/ttyUSB0 可能被占住，
           下次启动报「设备忙」，你会以为是权限问题查半天）
        3. 等读线程收尾（join 一下，别让它在你关串口后还在读）
        4. super().destroy_node()
        """
        # TODO 实现
        try:
            self._closing = True
        except Exception:
            pass
        
        super().destroy_node()

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

```

