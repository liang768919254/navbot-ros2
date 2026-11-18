// navbot_chassis 的协议层 —— 纯逻辑，零 ROS 依赖。
//
// 为什么单独拆出来：
//   帧解析/组帧/运动学是"给定输入必有确定输出"的纯函数，
//   不需要串口、不需要 rclcpp、不需要真机，最适合单元测试。
//   节点类只负责「读串口 → 调这里 → 发话题」，它自己不该有业务逻辑。
//
// 协议契约：13_ROS2自主导航移动机器人/参考源码/firmware/navbot_protocol.md
//   §2 上行 O 帧（11 字段）
//   §3 下行 V 帧（4 字段）+ 差速运动学
//   §8 固定样例向量（golden vector，测试用例的唯一来源）
#ifndef NAVBOT_CHASSIS__PROTOCOL_HPP_
#define NAVBOT_CHASSIS__PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <string_view>

namespace navbot_chassis
{

/// 解析结果。为什么要分成四种而不是只返回 bool：
/// 「不是我的帧」和「格式坏了」在统计和日志上必须分开，否则排查时无法区分
/// 是串口噪点还是真的协议不匹配。
enum class ParseResult
{
  kOk,             ///< 解析成功
  kNotAFrame,      ///< 帧头不是 'O'（多半是噪声或下行回显）
  kTooFewFields,   ///< 字段数 < 11（半包被误当完整帧传进来了）
  kBadNumber,      ///< 某个整数字段解析失败
};

/// 上行 O 帧的语义化表示。
/// 单位约定：速度 mm/s、电压 mV、温度 0.01 ℃（与协议文档一致，不在这里做换算）。
struct OdomFrame
{
  uint32_t seq{0};
  int32_t enc_l{0};
  int32_t enc_r{0};
  int32_t vel_l{0};      ///< mm/s
  int32_t vel_r{0};      ///< mm/s
  int32_t pwm_l{0};      ///< -1000 ~ 1000
  int32_t pwm_r{0};
  uint32_t vbat_mv{0};
  uint32_t tick_ms{0};
  int32_t temp_c100{0};  ///< 2510 表示 25.10 ℃
};

/// 解析一行 O 帧。行尾的 '\n' / '\r' 允许存在，函数内部忽略。
/// 字段数 > 11 时只取前 11 个（协议 §7 扩展预留：前 11 个必须存在，多的忽略）。
///
/// TODO(你) 实现。判据见 test_protocol.cpp —— 那里每一条断言就是验收标准。
ParseResult parse_odom_frame(const std::string & line, OdomFrame & out);

/// 下行 V 帧的内容。
struct VelCmd
{
  uint32_t seq{0};
  int32_t vel_l{0};   ///< mm/s
  int32_t vel_r{0};   ///< mm/s
};

/// 组装 V 帧，返回值**必须含结尾的 '\n'**（协议 §3）。
/// 形如 "V,567,120,-120\n"
///
/// TODO(你) 实现。
std::string format_velocity_frame(const VelCmd & cmd);

/// 差速运动学：车体速度 → 左右轮线速度（协议 §3）。
///   v_l = v - ω·L/2
///   v_r = v + ω·L/2
/// 其中 L = 0.16 m（★ 必须与 URDF 的 wheel_sep 一致）。
/// 注意：转 mm/s 要**四舍五入**，不是截断 —— 截断会带来系统性偏慢。
///
/// TODO(你) 实现。
void diff_drive_to_wheel_speeds(
  double v_mps, double w_radps,
  int32_t & vel_l_mms, int32_t & vel_r_mms);

/// 轮距常量。改这个值之前先确认 URDF 同步改了。
constexpr double kWheelSeparationM = 0.16;

/// 流式帧读取器：把「串口随便切片的字节流」变成「一帧一帧的 OdomFrame」。
///
/// 为什么需要它：串口不保证一次给你一整帧。可能是
///   feed("O,1234,-582")  →  300 ms 后  feed("0,5815,...,2510\n")
/// 单帧解析函数看不到"后面还有没有"，所以需要一层带缓冲的状态。
///
/// 两个必须处理的情况（协议 §8.3）：
///   1. 半包：没有 '\n' 就先存着，不能当坏帧计数
///   2. 垃圾撑爆缓冲：连续 kMaxBufferBytes 字节都没出现 '\n' → 清空 + 计 1 坏帧
///      （不然一条永不结束的噪声会把缓冲撑爆，之后所有真帧都被吞掉，
///        症状是"车突然失联、重启就好"，极难查）
class FrameReader
{
public:
  /// 喂入任意长度的字节切片（可以只是半帧）
  void feed(std::string_view bytes);

  /// 取出一帧。缓冲区里有完整的一行就解析并返回 true；否则返回 false。
  /// ★ 必须循环调用直到返回 false，因为一次 feed 可能喂进来多帧。
  bool pop(OdomFrame & out);

  /// 累计的坏帧数（帧头不对 / 字段数不足 / 数字非法 / 缓冲溢出）
  uint32_t bad_frame_count() const {return bad_frame_;}

  /// 清空缓冲与计数（重连时用）
  void reset();

  static constexpr std::size_t kMaxBufferBytes = 512;

private:
  std::string buffer_;
  uint32_t bad_frame_{0};
};

}  // namespace navbot_chassis

#endif  // NAVBOT_CHASSIS__PROTOCOL_HPP_
