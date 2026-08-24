// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__QUARTER_CAR_MODEL_HPP_
#define SUSPENSION_SIM__QUARTER_CAR_MODEL_HPP_

#include <Eigen/Dense>

#include <string>

#include "suspension_sim/suspension_model.hpp"

namespace suspension_sim
{

/// @brief 二自由度四分之一车模型（簧上质量 + 簧下质量）
///
/// 状态向量 state_ = [xs, xus, xs_dot, xus_dot]
///   - xs       簧上质量位移 (m)
///   - xus      簧下质量位移 (m)
///   - xs_dot   簧上质量速度 (m/s)
///   - xus_dot  簧下质量速度 (m/s)
///
/// 动力学方程（两自由度悬架模型，参考 Simulink 模型）：
///   ms * xs_ddot  = -ks*(xs - xus) - cs*(xs_dot - xus_dot) + u
///   mus * xus_ddot =  ks*(xs - xus) + cs*(xs_dot - xus_dot) - kt*(xus - road) - u
///
/// 其中 u 为主动悬架控制力（正值作用于簧上质量向上、簧下质量向下，
/// 即推力方向）。当 u = 0 时退化为被动悬架。
/// 采用显式欧拉积分进行数值求解。
class QuarterCarModel2DOF : public SuspensionModel
{
public:
  QuarterCarModel2DOF(double ms, double mus, double ks, double cs, double kt)
  : ms_(ms), mus_(mus), ks_(ks), cs_(cs), kt_(kt), body_accel_(0.0)
  {
    reset();
  }

  /// 重置状态为零
  void reset() override
  {
    state_.setZero();
    body_accel_ = 0.0;
  }

  /// 以给定路面高度、主动控制力和步长更新一步（显式欧拉积分）
  void update(double road_height, double dt, double control_force = 0.0) override
  {
    const double xs = state_(0);
    const double xus = state_(1);
    const double xs_dot = state_(2);
    const double xus_dot = state_(3);

    // 状态方程（参考 Simulink 模型；control_force > 0 时为推力）
    const double xs_ddot =
      (-ks_ * (xs - xus) - cs_ * (xs_dot - xus_dot) + control_force) / ms_;
    const double xus_ddot =
      (ks_ * (xs - xus) + cs_ * (xs_dot - xus_dot) -
      kt_ * (xus - road_height) - control_force) / mus_;

    // 记录最近一步的车身加速度，供 getBodyAccel() 输出
    body_accel_ = xs_ddot;

    // 显式欧拉积分
    state_(2) += xs_ddot * dt;
    state_(3) += xus_ddot * dt;
    state_(0) += state_(2) * dt;
    state_(1) += state_(3) * dt;
  }

  // Getters
  Eigen::VectorXd getState() const override {return state_;}

  /// 最近一步的车身（簧上质量）加速度 (m/s^2)
  double getBodyAccel() const override {return body_accel_;}

  /// 状态向量的维数（4：xs, xus, xs_dot, xus_dot）
  std::size_t stateSize() const override {return 4u;}

  /// 模型名称（与 YAML 中 type 字段 / 工厂匹配）
  std::string name() const override {return "quarter_car_2dof";}

private:
  double ms_;
  double mus_;
  double ks_;
  double cs_;
  double kt_;

  Eigen::Vector4d state_;  // [xs, xus, xs_dot, xus_dot]
  double body_accel_;      // 最近一步的簧上质量加速度，用于输出
};

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__QUARTER_CAR_MODEL_HPP_
