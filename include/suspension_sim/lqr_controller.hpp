// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__LQR_CONTROLLER_HPP_
#define SUSPENSION_SIM__LQR_CONTROLLER_HPP_

#include <Eigen/Dense>

#include <cstddef>

namespace suspension_sim
{

/// @brief 离散时间线性二次型调节器（LQR）
///
/// 针对离散系统：
///   x_{k+1} = A * x_k + B * u_k
/// 求解离散代数 Riccati 方程（DARE）：
///   P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
/// 得到状态反馈增益：
///   K = (R + B^T P B)^{-1} B^T P A
/// 控制律：u = -K * (x - x_ref)
///
/// 注意：Eigen 本身不提供 Riccati 方程求解器（`care()` 是 MATLAB /
/// SciPy / ROS control_toolbox 的接口，且求解的是*连续*方程）。这里采用
/// DARE 的标准不动点迭代求解，迭代收敛后 K 为最优状态反馈增益。
class LqrController
{
public:
  /// @brief 构造并计算最优增益
  /// @param A 系统矩阵 (n x n)
  /// @param B 输入矩阵 (n x m)
  /// @param Q 状态加权矩阵 (n x n)，通常半正定对角阵
  /// @param R 输入加权矩阵 (m x m)，通常正定对角阵
  LqrController(
    const Eigen::MatrixXd & A,
    const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & R);

  /// 求解离散代数 Riccati 方程并计算最优反馈增益 K
  void computeK();

  /// 计算控制量 u = -K * (x - x_ref)（单输入，返回标量）
  /// @param x     当前状态向量 (n x 1)
  /// @param x_ref 参考状态向量 (n x 1)，默认全零
  /// @return 控制力 u (N)
  double computeForce(const Eigen::VectorXd & x, const Eigen::VectorXd & x_ref) const;

  // 访问器
  const Eigen::MatrixXd & A() const {return A_;}
  const Eigen::MatrixXd & B() const {return B_;}
  const Eigen::MatrixXd & Q() const {return Q_;}
  const Eigen::MatrixXd & R() const {return R_;}
  const Eigen::MatrixXd & K() const {return K_;}
  const Eigen::MatrixXd & P() const {return P_;}

private:
  Eigen::MatrixXd A_;  ///< 系统矩阵
  Eigen::MatrixXd B_;  ///< 输入矩阵
  Eigen::MatrixXd Q_;  ///< 状态加权矩阵
  Eigen::MatrixXd R_;  ///< 输入加权矩阵
  Eigen::MatrixXd K_;  ///< 最优反馈增益 (m x n)
  Eigen::MatrixXd P_;  ///< DARE 的解（代价矩阵）
};

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__LQR_CONTROLLER_HPP_
