// navbot_chassis · P1 真机底盘桥（C++ 版）
//
// 目标：用 rclcpp 重写 Python 版桥节点（navbot_bridge/chassis_bridge.py），行为逐字段一致。
//
// 三条硬要求（照抄 Python 版的 R1/R2/R3，这是这个节点的骨架）：
//   R1  读串口放独立线程，绝不放在 spin 线程里
//   R2  用「定时器 + 最新值」发布，不用「收到即发」
//   R3  三层异常：连接层重连 / 数据层静默计数 / 时效层暂停发布
//
// ★ 与 Python 版的一处已知差异（2026-11-17 发现，需你确认以哪个为准）：
//   Python 版 parse_odom_frame 要求**字段数严格等于 11**（多一个也算坏帧）；
//   协议文档 §8.3 却写「12 个字段 → 接受，忽略第 12 个（§7 扩展预留）」。
//   本实现遵循**协议文档**（§8.4 声明"样例表是唯一来源，代码注释不算"），
//   即接受 >11 字段并忽略多余。若以 Python 版为准，改 protocol.cpp 的
//   parse_odom_frame 里 n < 11 那处判断即可，测试用例 AcceptsExtraFieldsAndIgnoresThem 要同步改。
//
// 协议契约：13_ROS2自主导航移动机器人/参考源码/firmware/navbot_protocol.md
#include "navbot_chassis/chassis_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "navbot_chassis/protocol.hpp"

namespace navbot_chassis
{

namespace
{

// ---- 与 Python 版对齐的常量 ----
constexpr int kDefaultBaud = 115200;
constexpr double kDefaultWheelSeparation = 0.16;    // L，★ 必须与 URDF 一致
constexpr double kDefaultWheelRadius = 0.0325;      // r
constexpr double kDefaultEncoderPpr = 2112.0;
constexpr double kDefaultPublishHz = 50.0;
constexpr double kDefaultStaleTimeoutS = 0.5;       // 时效层阈值
constexpr double kDefaultWarnThrottleS = 5.0;
constexpr int32_t kMaxWheelMms = 1000;              // 下行限幅 ±1.0 m/s
constexpr double kReconnectDelayS = 1.0;

constexpr char kJointLeft[] = "left_wheel_joint";
constexpr char kJointRight[] = "right_wheel_joint";

/// 把整数波特率映射成 termios 的 Bxxx 常量。
/// 不是所有值都支持 —— 只列协议文档用到的与常见的几种。
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

}  // namespace

// ============================================================
// 构造：参数、发布者、订阅者、定时器、读线程
// ============================================================
ChassisNode::ChassisNode()
: rclcpp::Node("chassis_node")
{
  // ---------- TODO 1 · 参数声明 ----------
  // 默认值不写死真机路径：/dev/ttyUSB0 只是兜底，实际用 --ros-args -p port:=... 覆盖
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baudrate", kDefaultBaud);
  wheel_separation_ = declare_parameter<double>("wheel_separation", kDefaultWheelSeparation);
  wheel_radius_ = declare_parameter<double>("wheel_radius", kDefaultWheelRadius);
  encoder_ppr_ = declare_parameter<double>("encoder_ppr", kDefaultEncoderPpr);
  publish_hz_ = declare_parameter<double>("publish_rate_hz", kDefaultPublishHz);
  stale_timeout_s_ = declare_parameter<double>("data_timeout_s", kDefaultStaleTimeoutS);
  warn_throttle_s_ = declare_parameter<double>("warn_throttle_s", kDefaultWarnThrottleS);

  RCLCPP_INFO(
    get_logger(),
    "chassis_node 启动：port=%s baud=%d L=%.4f r=%.4f ppr=%.0f pub=%.1fHz",
    port_.c_str(), baud_, wheel_separation_, wheel_radius_, encoder_ppr_, publish_hz_);

  // ---------- 发布者 / 订阅者 / TF ----------
  // 队列深度 10：odom 是高频数据，但下游 Nav2/AMCL 允许丢旧帧
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {on_cmd_vel(msg);});

  // ---------- R2 · 定时器发布 ----------
  // ★ 用定时器而不是"收到帧就发"：串口抖动时收到的节奏不稳，
  //   直接转发会让 /odom 的频率跟着抖，Nav2 的控制器会跟着抖。
  const auto period = std::chrono::duration<double>(1.0 / std::max(publish_hz_, 1.0));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {on_publish();});

  // ---------- R1 · 读线程 ----------
  // ★ 必须在最后启动：先让发布者/定时器就绪，避免读线程先拿到数据却没人消费
  read_thread_ = std::thread([this]() {read_loop();});

  RCLCPP_INFO(get_logger(), "chassis_node 初始化完成，等待 %s 上的 O 帧", port_.c_str());
}

ChassisNode::~ChassisNode()
{
  closing_ = true;                 // 让读线程在下一次超时后退出
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  close_serial();
}

// ============================================================
// R1 · 串口读线程
//   唯一职责：字节流 → FrameReader → on_frame()
//   不解析、不发话题、不做任何可能阻塞超过 100ms 的事
// ============================================================
void ChassisNode::read_loop()
{
  FrameReader reader;

  while (!closing_) {
    // ---- 连接层第 1 道关：打开串口 ----
    if (!open_serial()) {
      // ★ 打不开就等一会再试，绝不抛异常退出。
      //   节点要能一直活着等设备插上 —— 否则 systemd 会把它判定为崩溃并重启循环。
      warn_throttled("打开串口 " + port_ + " 失败，将重试");
      std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(kReconnectDelayS * 1000)));
      continue;
    }

    reader.reset();               // 新连接，丢掉上一轮的残帧
    RCLCPP_INFO(get_logger(), "串口已打开：%s @ %d", port_.c_str(), baud_);

    // ---- 读取循环 ----
    uint8_t buf[256];
    while (!closing_) {
      const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));

      if (n > 0) {
        // 把字节喂给 FrameReader，它负责攒行、切帧、计坏帧
        reader.feed(std::string_view(reinterpret_cast<const char *>(buf), static_cast<size_t>(n)));

        OdomFrame f;
        // ★ 必须 while：一次 read 可能带来多帧，用 if 会漏
        while (reader.pop(f)) {
          on_frame(f);
        }
      } else if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;              // 超时/被信号打断，都不是错误
        }
        warn_throttled(std::string("读串口出错，将重连: ") + std::strerror(errno));
        break;                   // 落到外层重连
      }
      // n == 0：VTIME 超时到点，这一轮没数据，正常
    }

    // ---- 连接层第 2 道关：关干净再重连 ----
    close_serial();

    if (!closing_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(kReconnectDelayS * 1000)));   // 退一步，别把 CPU 打满
    }
  }

  RCLCPP_INFO(get_logger(), "读线程退出");
}

bool ChassisNode::open_serial()
{
  // O_NOCTTY：不让串口成为控制终端（否则 Ctrl+C 的 SIGINT 会发给串口设备）
  // 不加 O_NONBLOCK：靠 VTIME 做读超时，这样 read 会阻塞但最多 100ms，
  //   退出时不需要中断系统调用
  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return false;
  }

  ::cfmakeraw(&tty);                       // 原始模式：不做任何字符转换
  const speed_t spd = baud_to_speed(baud_);
  ::cfsetispeed(&tty, spd);
  ::cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD);         // 忽略调制解调器线，启用接收
  tty.c_cflag &= ~CRTSCTS;                 // 无硬件流控（协议 §6：8N1 无流控）
  tty.c_cflag &= ~CSTOPB;                  // 1 位停止位
  tty.c_cflag &= ~PARENB;                  // 无校验

  tty.c_cc[VMIN] = 0;                      // 不要求最小字节数
  tty.c_cc[VTIME] = 1;                     // 1 × 100ms 读超时 → read 最多阻塞 100ms

  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return false;
  }

  ::tcflush(fd, TCIFLUSH);                 // 丢掉打开瞬间的残帧，避免第一帧是半个

  std::lock_guard<std::mutex> lk(serial_mutex_);
  serial_fd_ = fd;
  return true;
}

void ChassisNode::close_serial()
{
  std::lock_guard<std::mutex> lk(serial_mutex_);
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ============================================================
// 数据层：好帧只更新「最新值」，绝不在这里打日志
//
// ★ Python 版的原话：「第 6 组判据是日志 < 500 行，20% 乱码下 warn 一次就爆」。
//   坏帧由 FrameReader 自己计数，本函数只处理好帧。
// ============================================================
void ChassisNode::on_frame(const OdomFrame & f)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(state_mutex_);
  latest_ = f;
  has_frame_ = true;
  last_recv_t_ = now;
  stale_warned_ = false;     // ★ 数据回来了，复位时效层警告标志
  ++good_frames_;
}

// ============================================================
// R2 · 定时器发布：快照 → 时效判定 → 积分 → 发 /odom + TF + /joint_states
// ============================================================
void ChassisNode::on_publish()
{
  // ---- 1) 取快照（锁只拿三样东西，拿到立刻放） ----
  OdomFrame f;
  bool has = false;
  std::chrono::steady_clock::time_point recv_t;
  bool stale_warned = false;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    f = latest_;
    has = has_frame_;
    recv_t = last_recv_t_;
    stale_warned = stale_warned_;
  }

  const auto now = std::chrono::steady_clock::now();
  const double age_s = std::chrono::duration<double>(now - recv_t).count();

  // ---- 2) 时效层：数据陈旧就**暂停发布**，绝不发旧值 ----
  //   为什么不发旧值：Nav2 会拿旧位姿继续规划，机器人会"以为自己在原地"，
  //   实际车已经滑走了 —— 这比停下来更危险。

  // ★ 必须区分两种状态，否则启动瞬间会误报：
  //   「从未收到过帧」时 last_recv_t_ 是默认构造的 epoch，now - recv_t 是个天文数字，
  //   节点起来 20ms 就会打印"超过 0.5s 没有新帧"——措辞和事实都不对。
  if (!has) {
    if (!stale_warned) {
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        stale_warned_ = true;
      }
      RCLCPP_INFO(get_logger(), "等待 %s 上的第一帧", port_.c_str());
    }
    return;
  }

  if (age_s > stale_timeout_s_) {
    if (!stale_warned) {
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        stale_warned_ = true;       // 只警告一次，不刷屏
      }
      warn_throttled("超过 " + std::to_string(stale_timeout_s_) + "s 没有新帧，暂停发布 /odom");
    }
    return;
  }

  // ---- 3) 里程计积分（本节点唯一的算法部分） ----
  // 3a. 首帧保护：没有上一帧就算不出增量，只记录不清零
  if (!has_prev_enc_) {
    has_prev_enc_ = true;
    prev_enc_l_ = f.enc_l;
    prev_enc_r_ = f.enc_r;
    prev_pub_t_ = now;
    publish_joint_states(f, 0.0, 0.0);
    return;
  }

  const double dt = std::chrono::duration<double>(now - prev_pub_t_).count();
  if (dt <= 0.0) {
    return;                       // 时间倒流/重复 tick 的极端情况，跳过这一拍
  }
  prev_pub_t_ = now;

  // 3b. 编码器增量 → 左右轮位移（米）
  const int32_t d_enc_l = f.enc_l - prev_enc_l_;
  const int32_t d_enc_r = f.enc_r - prev_enc_r_;
  prev_enc_l_ = f.enc_l;
  prev_enc_r_ = f.enc_r;

  const double m_per_pulse = (2.0 * M_PI * wheel_radius_) / encoder_ppr_;
  const double d_l = d_enc_l * m_per_pulse;
  const double d_r = d_enc_r * m_per_pulse;

  // 3c. 轮位移 → 车体位移（差速正解）
  const double d_center = (d_l + d_r) / 2.0;
  const double d_yaw = (d_r - d_l) / wheel_separation_;

  // 3d. 积分成位姿 —— 中点法：用半个 yaw 增量，直行时与欧拉法等价，
  //     转向时误差更小（欧拉法在曲线上会系统性外扩）
  const double yaw_mid = yaw_ + d_yaw / 2.0;
  x_ += d_center * std::cos(yaw_mid);
  y_ += d_center * std::sin(yaw_mid);
  yaw_ += d_yaw;

  // 3e. 反馈速度用 MCU 报的 vel_l/vel_r（不是编码器差分）—— 更平滑，且与固件一致
  const double v = (f.vel_l + f.vel_r) / 2000.0;                       // mm/s → m/s
  const double omega = (static_cast<double>(f.vel_r) - f.vel_l) / 1000.0 / wheel_separation_;

  // ---- 4) 发 /odom ----
  const auto stamp = now_ros();
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";                // ★ 一字不能错
  odom.child_frame_id = "base_footprint";       // ★ 一字不能错
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  // yaw → 四元数（绕 Z 轴）
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = omega;
  odom_pub_->publish(odom);

  // ---- 5) 发 TF：odom → base_footprint ----
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.rotation.z = odom.pose.pose.orientation.z;
  tf.transform.rotation.w = odom.pose.pose.orientation.w;
  tf_broadcaster_->sendTransform(tf);

  // ---- 6) 发 /joint_states ----
  publish_joint_states(
    f,
    (f.vel_l / 1000.0) / wheel_radius_,        // mm/s → rad/s
    (f.vel_r / 1000.0) / wheel_radius_);
}

// /joint_states：position 用编码器累计脉冲换算，不要用 0 糊过去
// （差速车的轮子关节唯一用途就是给 RViz 转起来；给 0 会让轮子永远不动）
void ChassisNode::publish_joint_states(
  const OdomFrame & f, double vel_l_rad_s, double vel_r_rad_s)
{
  const double m_per_pulse = (2.0 * M_PI * wheel_radius_) / encoder_ppr_;

  sensor_msgs::msg::JointState js;
  js.header.stamp = now_ros();
  js.name = {kJointLeft, kJointRight};
  js.position = {
    f.enc_l * m_per_pulse / wheel_radius_,      // 脉冲 → 米 → 弧度
    f.enc_r * m_per_pulse / wheel_radius_,
  };
  js.velocity = {vel_l_rad_s, vel_r_rad_s};
  joint_pub_->publish(js);
}

// ============================================================
// 下行：/cmd_vel → 差速逆解 → V 帧 → 串口
//
// ★ 这个回调跑在 spin 线程里：不许 sleep、不许重试等待、不许阻塞重连。
//   写失败只记一笔，交给读线程去重连。
// ============================================================
void ChassisNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // ---- 1) 逆解：(v, ω) → 左右轮 mm/s ----
  int32_t vel_l = 0;
  int32_t vel_r = 0;
  diff_drive_to_wheel_speeds(msg->linear.x, msg->angular.z, vel_l, vel_r);

  // ---- 2) 限幅：上游手滑的保险丝 ----
  vel_l = std::clamp(vel_l, -kMaxWheelMms, kMaxWheelMms);
  vel_r = std::clamp(vel_r, -kMaxWheelMms, kMaxWheelMms);

  // ---- 3) 组 V 帧 ----
  VelCmd cmd;
  cmd.seq = cmd_seq_;
  cmd_seq_ = (cmd_seq_ + 1) & 0xFFFFu;          // 16 位回绕，与 Python 版一致
  cmd.vel_l = vel_l;
  cmd.vel_r = vel_r;
  const std::string line = format_velocity_frame(cmd);

  // ---- 4) 写串口 ----
  // 串口不在（正在重连）→ 静默丢弃。这不是错误：车本来就该停。
  std::lock_guard<std::mutex> lk(serial_mutex_);
  if (serial_fd_ < 0) {
    return;
  }
  const ssize_t written = ::write(serial_fd_, line.data(), line.size());
  if (written < 0) {
    warn_throttled(std::string("下发 V 帧失败: ") + std::strerror(errno));
  }
}

// ============================================================
// 日志节流：连接层与时效层共用，warn_throttle_s_ 秒内最多一条
// ============================================================
void ChassisNode::warn_throttled(const std::string & msg)
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - last_warn_t_).count();
  if (elapsed < warn_throttle_s_) {
    return;
  }
  last_warn_t_ = now;
  RCLCPP_WARN(get_logger(), "%s", msg.c_str());
}

}  // namespace navbot_chassis

// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<navbot_chassis::ChassisNode>();
    rclcpp::spin(node);
  }                                  // ★ 先析构（关读线程），再 shutdown
  rclcpp::shutdown();
  return 0;
}
