// navbot_chassis · P1 的容器
//
// 目标：用 rclcpp 重写 Python 版桥节点，行为逐字段一致。
// 参考实现（先读它，画出数据流图，图画完才许写代码）：
//   ~/navbot_ws/src/navbot_bridge/navbot_bridge/chassis_bridge.py
// 协议契约：
//   ~/桌面/具身智能学习路线/13_ROS2自主导航移动机器人/参考源码/firmware/navbot_protocol.md
//
// 本文件只给骨架和判据。TODO 全部由你自己填——填完才算 P1 完成。

#include <memory>

#include <rclcpp/rclcpp.hpp>

class ChassisNode : public rclcpp::Node
{
public:
  ChassisNode()
  : rclcpp::Node("chassis_node")
  {
    // TODO 1 · 参数声明
    //   要声明的：串口设备路径、波特率、上行发布频率、数据超时阈值
    //   判据：ros2 param list /chassis_node 能看到全部参数，且每个都有合理默认值
    //   注意：默认值不许写死你自己的 /dev/ttyUSB0，用参数覆盖
  }

private:
  // TODO 2 · 串口打开与读线程
  //   判据：拔掉串口后节点不崩；日志里每秒最多一条重连提示（不许刷屏）
  //   注意：读串口必须在独立线程，不许堵住 executor

  // TODO 3 · 上行帧解析（O 帧）→ nav_msgs/Odometry + TF(odom -> base_footprint)
  //   判据：ros2 topic hz /odom 达到参数里的频率；
  //        fake_chassis.py 喂入后，字段与 Python 版逐一对得上（不只"看起来有数"）
  //   注意：时间戳用 now()，不要用帧里带的毫秒数直接塞

  // TODO 4 · 下行 /cmd_vel → V 帧
  //   判据：ros2 topic pub /cmd_vel 后，串口侧能抓到对应字节序列
  //   注意：订阅回调里只做组帧和写，不做阻塞等待

  // TODO 5 · 三层异常
  //   连接层：串口断了怎么重连
  //   数据层：帧校验失败怎么计数、多久算静默
  //   时效层：多久没数据要停止发布 /odom（不是继续发旧值）
  //   判据：三种异常各注入一次，日志能分别报出来，且不互相混淆
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisNode>());
  rclcpp::shutdown();
  return 0;
}
