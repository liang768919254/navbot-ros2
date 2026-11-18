// navbot_chassis 协议层单元测试
//
// 期望值的唯一来源：navbot_protocol.md §8「固定样例向量（两端互操作基准）」
// ★ 这些断言就是 P1 的验收判据。不要为了让测试变绿去改期望值。
//
// 运行：
//   colcon test --packages-select navbot_chassis --event-handlers console_direct+
#include <gtest/gtest.h>

#include <string>

#include "navbot_chassis/protocol.hpp"

using navbot_chassis::diff_drive_to_wheel_speeds;
using navbot_chassis::format_velocity_frame;
using navbot_chassis::FrameReader;
using navbot_chassis::OdomFrame;
using navbot_chassis::parse_odom_frame;
using navbot_chassis::ParseResult;
using navbot_chassis::VelCmd;

// ============================================================
// 1. 上行正常样例（协议 §8.1）
//    O,1234,-5820,5815,120,118,420,415,11850,184320,2510
// ============================================================
TEST(OdomParse, GoldenVectorFromProtocolDoc)
{
  OdomFrame f;
  const auto r = parse_odom_frame(
    "O,1234,-5820,5815,120,118,420,415,11850,184320,2510", f);

  ASSERT_EQ(r, ParseResult::kOk);

  EXPECT_EQ(f.seq, 1234u);
  EXPECT_EQ(f.enc_l, -5820);
  EXPECT_EQ(f.enc_r, 5815);      // 左负右正 = 原地左转的编码器特征
  EXPECT_EQ(f.vel_l, 120);
  EXPECT_EQ(f.vel_r, 118);
  EXPECT_EQ(f.pwm_l, 420);
  EXPECT_EQ(f.pwm_r, 415);
  EXPECT_EQ(f.vbat_mv, 11850u);  // 11.85 V
  EXPECT_EQ(f.tick_ms, 184320u);
  EXPECT_EQ(f.temp_c100, 2510);  // 25.10 ℃
}

// 行尾带 \n 也要能解析（串口 readline 留下的换行很常见）
TEST(OdomParse, AcceptsTrailingNewline)
{
  OdomFrame f;
  const auto r = parse_odom_frame(
    "O,1234,-5820,5815,120,118,420,415,11850,184320,2510\n", f);
  ASSERT_EQ(r, ParseResult::kOk);
  EXPECT_EQ(f.seq, 1234u);
}

// ============================================================
// 2. 坏帧（协议 §8.3）—— 每一条都要能区分开
// ============================================================
TEST(OdomParse, RejectsTooFewFields)
{
  OdomFrame f;
  // 字段数 < 11
  EXPECT_EQ(parse_odom_frame("O,1234,-5820", f), ParseResult::kTooFewFields);
}

TEST(OdomParse, RejectsNonNumericField)
{
  OdomFrame f;
  // 整数字段解析失败
  EXPECT_EQ(
    parse_odom_frame("O,1234,-abc,5815,120,118,420,415,11850,184320,2510", f),
    ParseResult::kBadNumber);
}

TEST(OdomParse, RejectsWrongHeader)
{
  OdomFrame f;
  // 帧头不是 O：噪声
  EXPECT_EQ(
    parse_odom_frame("~~~GARBAGE### not a frame", f),
    ParseResult::kNotAFrame);
}

TEST(OdomParse, AcceptsExtraFieldsAndIgnoresThem)
{
  OdomFrame f;
  // 12 个字段：前 11 个必须存在，多的忽略（§7 扩展预留）
  const auto r = parse_odom_frame(
    "O,1234,-5820,5815,120,118,420,415,11850,184320,2510,999", f);
  ASSERT_EQ(r, ParseResult::kOk);
  EXPECT_EQ(f.seq, 1234u);
  EXPECT_EQ(f.temp_c100, 2510);   // 第 12 个 999 不该覆盖任何字段
}

// ============================================================
// 3. 下行组帧（协议 §8.2）
// ============================================================
TEST(VelocityFormat, GoldenVectorFromProtocolDoc)
{
  VelCmd c;
  c.seq = 567;
  c.vel_l = 120;
  c.vel_r = -120;      // 原地左转
  EXPECT_EQ(format_velocity_frame(c), std::string("V,567,120,-120\n"));
}

TEST(VelocityFormat, HandlesNegativeAndZero)
{
  VelCmd c;
  c.seq = 1;
  c.vel_l = -350;
  c.vel_r = 0;
  EXPECT_EQ(format_velocity_frame(c), std::string("V,1,-350,0\n"));
}

// ============================================================
// 4. 差速运动学（协议 §3）
//    轮距 L = 0.16 m
// ============================================================
TEST(Kinematics, StraightForward)
{
  int32_t l = 0, r = 0;
  diff_drive_to_wheel_speeds(0.2, 0.0, l, r);   // 0.2 m/s 直行
  EXPECT_EQ(l, 200);
  EXPECT_EQ(r, 200);
}

TEST(Kinematics, PureRotationInPlace)
{
  int32_t l = 0, r = 0;
  diff_drive_to_wheel_speeds(0.0, 1.0, l, r);   // 1 rad/s 原地逆时针
  EXPECT_EQ(l, -80);    // -1.0 * 0.16/2 = -0.08 m/s
  EXPECT_EQ(r, 80);
}

TEST(Kinematics, StraightPlusTurn)
{
  int32_t l = 0, r = 0;
  diff_drive_to_wheel_speeds(0.2, 0.5, l, r);
  EXPECT_EQ(l, 160);    // 0.2 - 0.5*0.08 = 0.16
  EXPECT_EQ(r, 240);    // 0.2 + 0.5*0.08 = 0.24
}

/// ★ 这一条专门抓「用截断而不是四舍五入」的实现。
/// 0.2019 m/s = 201.9 mm/s：四舍五入得 202，截断得 201。
/// 单帧差 1 mm/s 无所谓，但它会系统性偏慢 —— 这就是协议里强调 lround 的原因。
TEST(Kinematics, RoundsInsteadOfTruncating)
{
  int32_t l = 0, r = 0;
  diff_drive_to_wheel_speeds(0.2019, 0.0, l, r);
  EXPECT_EQ(l, 202) << "用了截断而不是四舍五入";
  EXPECT_EQ(r, 202) << "用了截断而不是四舍五入";
}

// ============================================================
// TODO(你) 5：半包不能当坏帧
//
// 场景：一串字节 "O,1234,-582" 后面还没等到 '\n'。
// 期望：不解析、不计数，留在缓冲区等下一批。
//
// 这条需要一个**有状态的流式解析器**（因为单帧函数看不到"后面还有没有"）。
// 建议接口：
//     class FrameReader {
//       void feed(const std::string& bytes);          // 喂任意切片
//       bool pop(OdomFrame& out);                     // 取出一帧（有则返回 true）
//       uint32_t bad_frame_count() const;
//     };
//
// 判据：
//   reader.feed("O,1234,-582");  EXPECT_FALSE(reader.pop(f));
//   reader.feed("0,5815,120,118,420,415,11850,184320,2510\n");
//   EXPECT_TRUE(reader.pop(f));  EXPECT_EQ(f.seq, 1234u);
// ============================================================
TEST(FrameReader, HalfPacketIsBuffered)
{
  FrameReader reader;
  OdomFrame f;

  // 前半帧：没有 '\n'，不该解析，也**不该计入坏帧**（这不是坏帧，是还没收完）
  reader.feed("O,1234,-582");
  EXPECT_FALSE(reader.pop(f));
  EXPECT_EQ(reader.bad_frame_count(), 0u) << "半包不该计入坏帧";

  // 补上后半帧，拼成完整的 11 字段
  reader.feed("0,5815,120,118,420,415,11850,184320,2510\n");
  ASSERT_TRUE(reader.pop(f));
  EXPECT_EQ(f.seq, 1234u);
  EXPECT_EQ(f.enc_l, -5820);      // 证明前半帧的 "-582" 真的和后半帧拼上了
  EXPECT_EQ(f.temp_c100, 2510);
  EXPECT_EQ(reader.bad_frame_count(), 0u) << "整个过程不该产生坏帧";
}

// ============================================================
// 5b. 一次 feed 到达多帧 —— 调用方必须 while(pop) 而不是 if(pop)
//
// 真实场景：主线程卡了一下，串口缓冲里积了两三帧一起送来。
// 如果调用方只 if(pop) 一次，剩下的帧会被留到下一轮，看起来像"丢帧"。
// ============================================================
TEST(FrameReader, HandlesMultipleFramesInOneFeed)
{
  FrameReader reader;
  OdomFrame f;

  reader.feed(
    "O,1,0,0,100,100,0,0,8000,1000,2500\n"
    "O,2,0,0,200,200,0,0,8000,1020,2500\n");

  int count = 0;
  while (reader.pop(f)) {
    ++count;
  }
  EXPECT_EQ(count, 2) << "一次 feed 里的两帧都要被取出来";
}

// ============================================================
// 6. 垃圾数据不能撑爆缓冲
//
// 协议 §8.3 最后一条，也是最容易漏的一条：
//   「连续 512 字节都没有出现 '\n' → 清空缓冲，计 1 个坏帧」
// ============================================================
TEST(FrameReader, GarbageOverflowResetsBuffer)
{
  FrameReader reader;
  OdomFrame f;

  reader.feed(std::string(500, 'x'));
  EXPECT_EQ(reader.bad_frame_count(), 0u) << "500 < 512，还没溢出，不该计数";

  reader.feed(std::string(20, 'x'));    // 累计 520 > 512，且整段没有 '\n'
  EXPECT_EQ(reader.bad_frame_count(), 1u) << "溢出后应清空缓冲并计 1 个坏帧";

  // 缓冲已清空 —— 之后的正常帧必须还能解出来，证明不是"卡死"而是"清干净了"
  reader.feed("O,1234,-5820,5815,120,118,420,415,11850,184320,2510\n");
  ASSERT_TRUE(reader.pop(f)) << "溢出清空后应能恢复正常解析";
  EXPECT_EQ(f.seq, 1234u);
}
