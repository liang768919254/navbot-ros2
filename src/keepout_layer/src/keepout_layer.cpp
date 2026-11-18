#include "keepout_layer/keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace keepout_layer
{

void KeepoutLayer::onInitialize()
{
  // ★★ 不要在 yaml 里用 enabled 控制本层 —— 基类不读它（实测 enabled_ 恒为 false）。
  //   用 keepout_enabled，自己读自己管。
  declareParameter("keepout_enabled", rclcpp::ParameterValue(true));
  declareParameter("keepout_min_x", rclcpp::ParameterValue(-1.0));
  declareParameter("keepout_max_x", rclcpp::ParameterValue(1.0));
  declareParameter("keepout_min_y", rclcpp::ParameterValue(-1.0));
  declareParameter("keepout_max_y", rclcpp::ParameterValue(1.0));

  // ★ Layer::node_ 是 std::weak_ptr，不能直接 node_->get_parameter()。
  //   declareParameter 内部帮你 lock 了，自己读参数必须手动 lock。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("KeepoutLayer: 拿不到 node 句柄（节点已析构？）");
  }

  const std::string prefix = name_ + ".";
  node->get_parameter(prefix + "keepout_enabled", active_);
  node->get_parameter(prefix + "keepout_min_x", keepout_.x0);
  node->get_parameter(prefix + "keepout_max_x", keepout_.x1);
  node->get_parameter(prefix + "keepout_min_y", keepout_.y0);
  node->get_parameter(prefix + "keepout_max_y", keepout_.y1);

  // 参数写反了也不会崩，但会得到一块空矩形 —— 归一化一下更省事
  if (keepout_.x0 > keepout_.x1) {std::swap(keepout_.x0, keepout_.x1);}
  if (keepout_.y0 > keepout_.y1) {std::swap(keepout_.y0, keepout_.y1);}

  RCLCPP_INFO(
    logger_,
    "KeepoutLayer '%s'：%s，禁区 x[%.2f, %.2f] y[%.2f, %.2f]",
    name_.c_str(),
    active_ ? "启用" : "禁用",
    keepout_.x0, keepout_.x1, keepout_.y0, keepout_.y1);
}

void KeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!active_) {
    return;
  }

  // 只"并入"边界，不覆盖 —— 其它层也要更新它们的区域
  *min_x = std::min(*min_x, keepout_.x0);
  *min_y = std::min(*min_y, keepout_.y0);
  *max_x = std::max(*max_x, keepout_.x1);
  *max_y = std::max(*max_y, keepout_.y1);
}

void KeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!active_) {
    return;
  }

  const double res = master_grid.getResolution();
  const double ox = master_grid.getOriginX();
  const double oy = master_grid.getOriginY();

  // 世界坐标 → 栅格索引。★ 不直接用 worldToMap：
  //   禁区可能部分落在地图外，worldToMap 会直接返回 false 把整块丢掉。
  //   这里先算再 clamp，部分重叠的也能标上。
  int i0 = static_cast<int>(std::floor((keepout_.x0 - ox) / res));
  int i1 = static_cast<int>(std::floor((keepout_.x1 - ox) / res));
  int j0 = static_cast<int>(std::floor((keepout_.y0 - oy) / res));
  int j1 = static_cast<int>(std::floor((keepout_.y1 - oy) / res));

  if (i0 > i1) {std::swap(i0, i1);}
  if (j0 > j1) {std::swap(j0, j1);}

  // ★★ 先与本帧允许更新的窗口求交 —— 越界写会破坏其它层的更新。
  //    Nav2 的惯例是 [min, max) 左闭右开。
  i0 = std::max(i0, min_i);
  j0 = std::max(j0, min_j);
  i1 = std::min(i1, max_i - 1);
  j1 = std::min(j1, max_j - 1);

  // 再与地图边界求交
  i0 = std::max(i0, 0);
  j0 = std::max(j0, 0);
  i1 = std::min(i1, static_cast<int>(master_grid.getSizeInCellsX()) - 1);
  j1 = std::min(j1, static_cast<int>(master_grid.getSizeInCellsY()) - 1);

  if (i0 > i1 || j0 > j1) {
    return;   // 禁区完全不在地图/窗口内
  }

  unsigned int cells = 0;
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      master_grid.setCost(
        static_cast<unsigned int>(i),
        static_cast<unsigned int>(j),
        nav2_costmap_2d::LETHAL_OBSTACLE);
      ++cells;
    }
  }

  if (!logged_once_ && cells > 0) {
    logged_once_ = true;
    RCLCPP_INFO(
      logger_,
      "KeepoutLayer 首次生效：%u 个格子标记为致命障碍（i %d..%d, j %d..%d）",
      cells, i0, i1, j0, j1);
  }
}

}  // namespace keepout_layer

// ★ 让 pluginlib 认识这个类。第二个参数必须是基类全名。
PLUGINLIB_EXPORT_CLASS(keepout_layer::KeepoutLayer, nav2_costmap_2d::Layer)
