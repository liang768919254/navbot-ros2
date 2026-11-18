// 协议层的实现。
//
// ★ 关 2 / 3 / 4 由 AI 代写（2026-11-17）。关 1 是他自己写的。
//   注释写得比平时密，目的：这份代码可以当教材读。
//   但记住一件事——**读懂了 ≠ 写得出**。要证明掌握，见 README 的「闭卷重写清单」。
#include "navbot_chassis/protocol.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <system_error>

namespace navbot_chassis
{

namespace
{

/// 把一段文本转成 uint32_t，且要求**整个字段都被消费掉**。
///
/// 为什么要检查 ptr == end：
///   std::from_chars 遇到 "12abc" 会成功转换 12，ptr 停在 'a'。
///   不检查就等于把 "12abc" 当成 12 —— 串口噪声里这种"数字粘字母"极其常见，
///   一旦放过，坏帧会被当成好帧，里程计会突然跳一下。
///   这也是本实现不用 std::stoi 的原因：stoi("12abc") 直接返回 12，不报错。
bool parse_u32(std::string_view sv, uint32_t & v)
{
  if (sv.empty()) {
    return false;
  }
  const char * begin = sv.data();
  const char * end = begin + sv.size();
  const auto res = std::from_chars(begin, end, v);
  // ec 为 errc{} 表示无错误；ptr 必须走到末尾
  return res.ec == std::errc{} && res.ptr == end;
}

/// 有符号版本。from_chars 对 int32_t 能正确接受前导 '-'，
/// 但对 uint32_t 遇到 '-' 会返回 invalid_argument —— 正好帮我们挡住负数喂给 seq。
bool parse_i32(std::string_view sv, int32_t & v)
{
  if (sv.empty()) {
    return false;
  }
  const char * begin = sv.data();
  const char * end = begin + sv.size();
  const auto res = std::from_chars(begin, end, v);
  return res.ec == std::errc{} && res.ptr == end;
}

}  // namespace

ParseResult parse_odom_frame(const std::string & line, OdomFrame & out)
{
  // ---------- ① 剥掉首尾空白 ----------
  // 串口 readline 常常带 '\r'（Windows 风格换行）或多余空格。
  // 协议说以 '\n' 结尾，但容忍 '\r' 的成本几乎为零，容错换来的稳定性很值。
  const auto first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return ParseResult::kNotAFrame;   // 整行都是空白
  }
  const auto last = line.find_last_not_of(" \t\r\n");
  const std::string_view s(line.data() + first, last - first + 1);

  // ---------- ② 帧头检查 ----------
  // 先查帧头再做切分：噪声行可能很长，早点返回省事。
  if (s.front() != 'O') {
    return ParseResult::kNotAFrame;
  }

  // ---------- ③ 按 ',' 切分 ----------
  // 只需要前 11 个字段（f[0] 是 "O"，f[1..10] 是 10 个数字）。
  // 协议 §7 扩展预留：前 11 个必须存在，多的忽略 → 所以数组开到 12 就够，
  // 第 12 个字段（如果存在）读进来也无所谓，只要不参与赋值。
  std::string_view f[11];
  std::size_t n = 0;
  std::size_t pos = 0;
  while (n < 11) {
    const std::size_t comma = s.find(',', pos);
    if (comma == std::string_view::npos) {
      f[n++] = s.substr(pos);          // 最后一个字段，没有逗号
      break;
    }
    f[n++] = s.substr(pos, comma - pos);
    pos = comma + 1;
  }
  if (n < 11) {
    return ParseResult::kTooFewFields;
  }

  // ---------- ④ 逐字段转换 ----------
  // 先写进临时对象：任何一个字段失败就整体丢弃，不污染调用方的 out。
  // 这个习惯很重要 —— 否则调用方会拿到"一半新一半旧"的脏数据。
  OdomFrame tmp;

  if (!parse_u32(f[1], tmp.seq))       {return ParseResult::kBadNumber;}
  if (!parse_i32(f[2], tmp.enc_l))     {return ParseResult::kBadNumber;}
  if (!parse_i32(f[3], tmp.enc_r))     {return ParseResult::kBadNumber;}
  if (!parse_i32(f[4], tmp.vel_l))     {return ParseResult::kBadNumber;}
  if (!parse_i32(f[5], tmp.vel_r))     {return ParseResult::kBadNumber;}
  if (!parse_i32(f[6], tmp.pwm_l))     {return ParseResult::kBadNumber;}
  if (!parse_i32(f[7], tmp.pwm_r))     {return ParseResult::kBadNumber;}
  if (!parse_u32(f[8], tmp.vbat_mv))   {return ParseResult::kBadNumber;}
  if (!parse_u32(f[9], tmp.tick_ms))   {return ParseResult::kBadNumber;}
  if (!parse_i32(f[10], tmp.temp_c100)) {return ParseResult::kBadNumber;}

  out = tmp;
  return ParseResult::kOk;
}

std::string format_velocity_frame(const VelCmd & cmd)
{
  // 直接拼。这里是热路径（20 Hz），用 std::to_string + operator+= 足够快，
  // 上 stringstream 反而慢一个数量级（它要构造 sentry、处理 locale）。
  std::string s;
  s.reserve(32);                     // 预留，避免多次 realloc

  s += 'V';
  s += ',';
  s += std::to_string(cmd.seq);
  s += ',';
  s += std::to_string(cmd.vel_l);
  s += ',';
  s += std::to_string(cmd.vel_r);
  s += '\n';                         // ★ 结尾必须有，否则 MCU 侧一直等下一批

  return s;
}

void diff_drive_to_wheel_speeds(
  double v_mps, double w_radps,
  int32_t & vel_l_mms, int32_t & vel_r_mms)
{
  // 半轮距 = L/2。用常量而不是字面量 0.08 —— 将来标定出真实轮距，只改一处。
  const double half = kWheelSeparationM / 2.0;

  // 差速运动学（协议 §3）：右转时左轮慢、右轮快，所以 ω 在左轮取负、右轮取正。
  const double v_l = v_mps - w_radps * half;
  const double v_r = v_mps + w_radps * half;

  // ★ 必须 lround（四舍五入），不能 static_cast 截断。
  //   截断会让每一拍都少 0~1 mm/s，是**单向**的系统性偏差 —— PID 补不回来，
  //   跑 10 米会差几十厘米。这就是协议文档里那句"截断会带来系统性偏慢"。
  //   注意 lround 返回 long，赋给 int32_t 要显式转换，避免 -Wconversion 警告。
  vel_l_mms = static_cast<int32_t>(std::lround(v_l * 1000.0));
  vel_r_mms = static_cast<int32_t>(std::lround(v_r * 1000.0));
}

// ============================================================
// FrameReader：把字节流切成帧
// ============================================================

void FrameReader::feed(std::string_view bytes)
{
  buffer_.append(bytes.data(), bytes.size());

  // 垃圾保护：缓冲超过上限且**整段没有换行** → 判定为永不结束的噪声。
  //
  // 为什么条件要带 "没有换行"：
  //   如果缓冲里已经有换行，说明只是"一次喂多了"，pop 会正常消费掉，
  //   不该判坏帧。只有"攒了一大堆却一个换行都没有"才是真垃圾。
  //
  // 为什么必须做这件事：不做的话，一条永不结束的噪声会把缓冲无限撑大，
  //   之后所有真帧都排在它后面，永远解析不出来 —— 表现为"车突然失联"。
  if (buffer_.size() > kMaxBufferBytes &&
    buffer_.find('\n') == std::string::npos)
  {
    buffer_.clear();
    ++bad_frame_;
  }
}

bool FrameReader::pop(OdomFrame & out)
{
  // 没有换行 = 半包，留在缓冲里等下一批。**不计数**（这不是坏帧，是还没收完）。
  const std::size_t nl = buffer_.find('\n');
  if (nl == std::string::npos) {
    return false;
  }

  // 取出这一行（不含 '\n'），并从缓冲里移除
  const std::string line = buffer_.substr(0, nl);
  buffer_.erase(0, nl + 1);

  const ParseResult r = parse_odom_frame(line, out);
  if (r != ParseResult::kOk) {
    // 这一行是坏的：计数，返回 false。
    // 注意返回 false 不代表"没数据了"——缓冲里可能还有下一行，
    // 所以调用方必须写成 while (reader.pop(f)) { ... } 而不是 if。
    ++bad_frame_;
    return false;
  }
  return true;
}

void FrameReader::reset()
{
  buffer_.clear();
  bad_frame_ = 0;
}

}  // namespace navbot_chassis
