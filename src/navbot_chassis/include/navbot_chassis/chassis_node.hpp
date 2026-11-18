// chassis_node 的类声明。
//
// 为什么把声明和实现拆开：这个节点的实现有 300+ 行，混在一起读起来累。
// 更重要的是——声明里能一眼看全「它持有哪些状态」，实现里只剩「怎么用这些状态」。
#ifndef NAVBOT_CHASSIS__CHASSIS_NODE_HPP_
#define NAVBOT_CHASSIS__CHASSIS_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "navbot_chassis/protocol.hpp"

namespace navbot_chassis
{

class ChassisNode : public rclcpp::Node
{
public:
  ChassisNode();
  ~ChassisNode() override;

  // 不允许拷贝：内部有线程和 mutex
  ChassisNode(const ChassisNode &) = delete;
  ChassisNode & operator=(const ChassisNode &) = delete;

private:
  // ---- R1 读线程 ----
  void read_loop();
  bool open_serial();
  void close_serial();

  // ---- 数据层 ----
  void on_frame(const OdomFrame & f);

  // ---- R2 定时发布 ----
  void on_publish();
  void publish_joint_states(const OdomFrame & f, double vel_l_rad_s, double vel_r_rad_s);

  // ---- 下行 ----
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

  // ---- 工具 ----
  void warn_throttled(const std::string & msg);
  rclcpp::Time now_ros() {return this->now();}

  // ============ 参数 ============
  std::string port_;
  int baud_{115200};
  double wheel_separation_{0.16};
  double wheel_radius_{0.0325};
  double encoder_ppr_{2112.0};
  double publish_hz_{50.0};
  double stale_timeout_s_{0.5};
  double warn_throttle_s_{5.0};

  // ============ 串口 / 线程 ============
  int serial_fd_{-1};                     // -1 表示未打开；受 serial_mutex_ 保护
  mutable std::mutex serial_mutex_;
  std::atomic<bool> closing_{false};
  std::thread read_thread_;

  // ============ 共享状态（读线程写，定时器读）============
  mutable std::mutex state_mutex_;
  OdomFrame latest_{};
  bool has_frame_{false};
  std::chrono::steady_clock::time_point last_recv_t_{};
  bool stale_warned_{false};
  uint64_t good_frames_{0};

  // ============ 里程计（只在定时器线程里改，不需要锁）============
  bool has_prev_enc_{false};
  int32_t prev_enc_l_{0};
  int32_t prev_enc_r_{0};
  std::chrono::steady_clock::time_point prev_pub_t_{};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  // ============ ROS 接口 ============
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ============ 其他（只在单线程里改）============
  uint32_t cmd_seq_{0};                   // 只在 cmd_vel 回调里改（spin 线程）
  std::chrono::steady_clock::time_point last_warn_t_{};   // 多线程访问，但只用于节流
};

}  // namespace navbot_chassis

#endif  // NAVBOT_CHASSIS__CHASSIS_NODE_HPP_
