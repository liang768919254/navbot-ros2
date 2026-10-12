# SensorData.msg —— 一帧传感器数据（与串口协议 v3 一一对应）
#
# ══════════════════════════════════════════════════════════════
# 为什么长这样（三条设计决定）
# ══════════════════════════════════════════════════════════════
#
# ① 用 std_msgs/Header，而不是自己写 stamp / frame_id
#    · header.stamp 是 builtin_interfaces/Time（int32 sec + uint32 nanosec），
#      bag 回放、rviz、tf2、image_transport 全都认它。
#      自己塞一个 float64 秒：精度会在长时间轴上丢，而且没有任何工具认。
#    · header.frame_id 是 TF2 查坐标变换用的话题名，位置是生态约定好的。
#    结业清单最初写的是 `float64 stamp` / `string frame_id` 两个裸字段 ——
#    功能等价，但等于自己造轮子，后面接 TF2 时要全部返工。
#
# ② 字段顺序 = 语义分组：时间 → 身份 → 主测量 → 原始量 → 设备时钟
#    顺序本身不影响功能，但影响所有人读代码时的第一印象。
#
# ③ 类型挑最小的够用
#    ADC 是 12 位 → uint16 就够，不必 int32。
#    字段类型决定了 DDS 序列化后的字节数，也决定了以后多传感器合流时的带宽。
# ══════════════════════════════════════════════════════════════

std_msgs/Header header      # stamp=本机发布的时刻（到达时刻）；frame_id=安装坐标名
uint32 seq                  # MCU 侧帧号；订阅端用它检测丢帧（不要用 ROS 的 header.seq，那个已废弃）
float32 value               # 主工程量（与 value_scale 参数配套，默认单位 mV）
float32 temperature         # 温度，单位 ℃
uint16 raw_adc              # ADC 原始值 0~4095
uint32 mcu_tick_ms          # MCU 上电以来的毫秒数，用于事后把"到达时刻"校正成"采样时刻"
