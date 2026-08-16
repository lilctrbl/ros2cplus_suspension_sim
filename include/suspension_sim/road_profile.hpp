// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__ROAD_PROFILE_HPP_
#define SUSPENSION_SIM__ROAD_PROFILE_HPP_

#include <cmath>
#include <cstdint>
#include <random>
#include <string>

namespace suspension_sim
{

/// @brief 路面模型抽象基类（策略模式）
///
/// 每种路面实现为一个派生类，节点只需持有一个 RoadProfile 指针即可
/// 在运行时切换不同的路面输入。
class RoadProfile
{
public:
  virtual ~RoadProfile() = default;

  /// 返回 t 时刻的路面高度 y（单位 m）
  virtual double compute_height(double t) const = 0;

  /// 路面名称，用于日志 / 调试
  virtual std::string name() const = 0;
};

/// @brief 正弦路面: y = A * sin(2 * PI * f * t)
class SineRoadProfile : public RoadProfile
{
public:
  SineRoadProfile(double amplitude, double frequency)
  : amplitude_(amplitude), frequency_(frequency) {}

  double compute_height(double t) const override
  {
    return amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);
  }

  std::string name() const override { return "sine"; }

private:
  double amplitude_;
  double frequency_;
};

/// @brief 方波路面: 周期 T，高电平幅值 +A，低电平幅值 -A
class SquareRoadProfile : public RoadProfile
{
public:
  SquareRoadProfile(double amplitude, double frequency)
  : amplitude_(amplitude), frequency_(frequency) {}

  double compute_height(double t) const override
  {
    const double period = 1.0 / frequency_;
    const double half = period / 2.0;
    const double phase = std::fmod(t, period);
    return (phase < half) ? amplitude_ : -amplitude_;
  }

  std::string name() const override { return "square"; }

private:
  double amplitude_;
  double frequency_;
};

/// @brief 随机路面: 以给定采样间隔在 [-amplitude, +amplitude] 内随机跳变
class RandomRoadProfile : public RoadProfile
{
public:
  RandomRoadProfile(double amplitude, double sample_interval, uint32_t seed = 0u)
  : amplitude_(amplitude),
    sample_interval_(sample_interval),
    engine_(seed),
    distribution_(-amplitude_, amplitude_),
    last_sample_time_(0.0),
    last_value_(distribution_(engine_))
  {}

  double compute_height(double t) const override
  {
    // 超过采样间隔则重新抽样（mutable 允许在 const 查询中更新状态）
    if (t - last_sample_time_ >= sample_interval_) {
      last_sample_time_ = t;
      last_value_ = distribution_(engine_);
    }
    return last_value_;
  }

  std::string name() const override { return "random"; }

private:
  double amplitude_;
  double sample_interval_;
  mutable std::mt19937 engine_;
  mutable std::uniform_real_distribution<double> distribution_;
  mutable double last_sample_time_;
  mutable double last_value_;
};

/// @brief 减速带路面: 单次三角形凸起，之后保持平路
class SpeedBumpRoadProfile : public RoadProfile
{
public:
  SpeedBumpRoadProfile(double height, double duration)
  : height_(height), duration_(duration) {}

  double compute_height(double t) const override
  {
    if (t >= 0.0 && t < duration_) {
      // 上升沿 + 下降沿的三角波
      return height_ * std::min(2.0 * t / duration_, 2.0 - 2.0 * t / duration_);
    }
    return 0.0;
  }

  std::string name() const override { return "speed_bump"; }

private:
  double height_;
  double duration_;
};

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__ROAD_PROFILE_HPP_
