// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__SUSPENSION_MODEL_HPP_
#define SUSPENSION_SIM__SUSPENSION_MODEL_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>

namespace suspension_sim
{

/// @brief 悬架模型抽象基类
///
/// 定义所有悬架动力学模型共有的接口，节点只需持有
/// `std::unique_ptr<SuspensionModel>` 即可统一驱动任意具体模型，
/// 便于后续扩展更多自由度 / 更复杂的模型。
class SuspensionModel
{
public:
  virtual ~SuspensionModel() = default;

  /// 重置状态为零
  virtual void reset() = 0;

  /// 以给定路面高度、主动控制力和步长更新一步（由具体模型决定积分方式）
  virtual void update(double road_height, double dt, double control_force = 0.0) = 0;

  /// 当前状态向量（各模型的维度可能不同，见 stateSize()）
  virtual Eigen::VectorXd getState() const = 0;

  /// 最近一步的车身（簧上质量）加速度 (m/s^2)
  virtual double getBodyAccel() const = 0;

  /// 状态向量的维数
  virtual std::size_t stateSize() const = 0;

  /// 模型名称，用于日志 / 调试 / 工厂匹配
  virtual std::string name() const = 0;
};

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__SUSPENSION_MODEL_HPP_
