// Copyright 2026
// Apache-2.0

#include "suspension_sim/kalman_filter.hpp"

#include <stdexcept>

namespace suspension_sim
{

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A,
  const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & Q_proc,
  const Eigen::MatrixXd & R_meas)
: A_(A), B_(B), C_(C), Q_proc_(Q_proc), R_meas_(R_meas)
{
  // 维度一致性检查（错误配置尽早暴露，避免运行时断言）
  const Eigen::Index n = A_.rows();
  const Eigen::Index m = B_.cols();
  const Eigen::Index p = C_.rows();
  if (A_.cols() != n) {
    throw std::invalid_argument("KalmanFilter: A must be square (n x n)");
  }
  if (B_.rows() != n) {
    throw std::invalid_argument("KalmanFilter: B must be (n x m)");
  }
  if (C_.cols() != n) {
    throw std::invalid_argument("KalmanFilter: C must be (p x n)");
  }
  if (Q_proc_.rows() != n || Q_proc_.cols() != n) {
    throw std::invalid_argument("KalmanFilter: Q_proc must be (n x n)");
  }
  if (R_meas_.rows() != p || R_meas_.cols() != p) {
    throw std::invalid_argument("KalmanFilter: R_meas must be (p x p)");
  }
  (void)m;  // B 的列数用于确定控制输入维度，此处仅一致性约束

  reset();
}

void KalmanFilter::predict(const Eigen::VectorXd & u)
{
  // 控制输入：空向量视为 0，否则需与 B 列数一致
  if (u.size() != 0) {
    if (u.size() != B_.cols()) {
      throw std::invalid_argument("KalmanFilter::predict: u size mismatch");
    }
    x_hat_ = A_ * x_hat_ + B_ * u;
  } else {
    x_hat_ = A_ * x_hat_;
  }

  // P^- = A * P * A^T + Q_proc
  P_ = A_ * P_ * A_.transpose() + Q_proc_;
}

void KalmanFilter::update(const Eigen::VectorXd & y)
{
  if (y.size() != C_.rows()) {
    throw std::invalid_argument("KalmanFilter::update: y size mismatch");
  }

  // 卡尔曼增益：K = P^- * C^T * (C * P^- * C^T + R_meas)^{-1}
  const Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_meas_;
  const Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

  // 状态修正：x_hat = x_hat^- + K * (y - C * x_hat^-)
  x_hat_ += K * (y - C_ * x_hat_);

  // 协方差修正（Joseph 形式，数值更稳定）：P = (I - K*C) * P^- * (I - K*C)^T + K*R*K^T
  const Eigen::Index n = A_.rows();
  const Eigen::MatrixXd IKC = Eigen::MatrixXd::Identity(n, n) - K * C_;
  P_ = IKC * P_ * IKC.transpose() + K * R_meas_ * K.transpose();
}

void KalmanFilter::step(const Eigen::VectorXd & y, const Eigen::VectorXd & u)
{
  predict(u);
  update(y);
}

void KalmanFilter::reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
  x_hat_ = x0;
  P_ = P0;
}

void KalmanFilter::reset()
{
  const Eigen::Index n = A_.rows();
  // 默认初始均值全零；初始协方差取小量对角阵（认为初值近似已知）
  x_hat_ = Eigen::VectorXd::Zero(n);
  P_ = Eigen::MatrixXd::Identity(n, n) * 1e-2;
}

}  // namespace suspension_sim
