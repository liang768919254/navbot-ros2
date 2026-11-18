// 自定义 costmap 层：一块**固定世界坐标**的禁入区。
//
// 为什么是固定坐标而不是"跟着机器人走"：
//   禁区语义是「这里业务上不能进」（地毯边、充电桩前方、门口堆物），
//   跟着机器人走的那是"安全边界"，性质不同。固定矩形才能演示"规划绕开"。
//
// 用法：写进 costmap 的 plugins 列表，四个角用参数给（米，map 坐标系）。
//
// ★ 两个必须做对的地方（做错了表现为「插件加载了但没效果」）：
//   1. updateBounds 只扩边界、不写值；写值在 updateCosts
//   2. updateCosts 只能改 [min_i,min_j]..[max_i,max_j] 窗口内的格子，
//      越界会破坏其它层的更新
#ifndef KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_
#define KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace keepout_layer
{

class KeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  KeepoutLayer() = default;

  void onInitialize() override;

  /// 只扩边界，不写代价
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  /// 在允许的窗口内把禁区格子写成致命障碍
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override
  {
    // 本层无累积状态（禁区来自静态参数），不用清理。
    // 如果有缓存（比如点云累积），这里必须清，否则地图会"记住"旧障碍。
  }

  bool isClearable() override {return false;}
  // ★ false 是有意的：禁区是业务约束，不该被 clear_costmap 清掉。
  //   返回 true 的话，Nav2 的恢复行为 ClearEntireCostmap 会把它抹掉，
  //   表现为「第一次导航绕开了，触发一次恢复之后就直接穿过去」——极难查。

private:
  struct Rect
  {
    double x0{0.0};
    double x1{0.0};
    double y0{0.0};
    double y1{0.0};
  };

  Rect keepout_;
  bool logged_once_{false};    // 只在首次生效时打日志，避免每帧刷屏

  // ★★ 开关必须自己管（2026-11-18 实测踩坑）：
  //   Humble 的 nav2_costmap_2d::Layer 基类**不会**自动读 "<name>.enabled"，
  //   它的 enabled_ 成员是未初始化的（实测为 false）。
  //   所以 yaml 里写 enabled: true 完全没用，日志会显示"禁用"、层不生效。
  //   这里用独立参数名 keepout_enabled，避免与基类同名参数冲突。
  bool active_{false};
};

}  // namespace keepout_layer

#endif  // KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_
