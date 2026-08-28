// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__KALMAN_FILTER_HPP_
#define SUSPENSION_SIM__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace suspension_sim
{

/// @brief 离散时间卡尔曼滤波器（标准 Kalman Filter）
///
/// 面向线性离散系统：
///   x_{k+1} = A * x_k + B * u_k + w_k,   w ~ N(0, Q_proc)
///   y_k      = C * x_k + v_k,            v ~ N(0, R_meas)
///
/// 分两步递归估计状态均值 x_hat 与误差协方差 P：
///   预测：
///     x_hat^- = A * x_hat + B * u
///     P^-     = A * P * A^T + Q_proc
///   更新：
///     K       = P^- * C^T * (C * P^- * C^T + R_meas)^{-1}
///     x_hat   = x_hat^- + K * (y - C * x_hat^-)
///     P       = (I - K * C) * P^-
///
/// 提供 reset() 重新初始化状态均值 / 协方差。
class KalmanFilter
{
public:
  /// @brief 构造滤波器
  /// @param A       系统矩阵 (n x n)
  /// @param B       输入矩阵 (n x m)
  /// @param C       观测矩阵 (p x n)
  /// @param Q_proc  过程噪声协方差 (n x n)，通常半正定对角阵
  /// @param R_meas  观测噪声协方差 (p x p)，通常正定对角阵
  KalmanFilter(
    const Eigen::MatrixXd & A,
    const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & Q_proc,
    const Eigen::MatrixXd & R_meas);

  /// @brief 预测步：x_hat <- A*x_hat + B*u；P <- A*P*A^T + Q_proc
  /// @param u 控制输入向量 (m x 1)
  void predict(const Eigen::VectorXd & u = Eigen::VectorXd());

  /// @brief 更新步：结合新观测 y 修正 x_hat 与 P
  /// @param y 观测向量 (p x 1)
  void update(const Eigen::VectorXd & y);

  /// @brief 便捷接口：一步预测 + 一步更新（等价于依次调用 predict/update）
  /// @param y 观测向量 (p x 1)
  /// @param u 控制输入向量 (m x 1)，默认空向量（视为 0）
  void step(const Eigen::VectorXd & y, const Eigen::VectorXd & u = Eigen::VectorXd());

  /// @brief 将状态均值 / 协方差重置为给定值
  /// @param x0 初始状态均值 (n x 1)
  /// @param P0 初始协方差 (n x n)，通常取较大的对角阵表示初始不确定
  void reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);

  /// 重置为全零状态与默认协方差（单位阵 * 1e-2）
  void reset();

  // 访问器
  const Eigen::VectorXd & xHat() const {return x_hat_;}
  const Eigen::MatrixXd & P() const {return P_;}

private:
  Eigen::MatrixXd A_;       ///< 系统矩阵 (n x n)
  Eigen::MatrixXd B_;       ///< 输入矩阵 (n x m)
  Eigen::MatrixXd C_;       ///< 观测矩阵 (p x n)
  Eigen::MatrixXd Q_proc_;  ///< 过程噪声协方差 (n x n)
  Eigen::MatrixXd R_meas_;  ///< 观测噪声协方差 (p x p)
  Eigen::VectorXd x_hat_;   ///< 状态均值估计 (n x 1)
  Eigen::MatrixXd P_;       ///< 误差协方差 (n x n)
};

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__KALMAN_FILTER_HPP_
