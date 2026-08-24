// Copyright 2026
// Apache-2.0

#include "suspension_sim/lqr_controller.hpp"

#include <cmath>

namespace suspension_sim
{

LqrController::LqrController(
  const Eigen::MatrixXd & A,
  const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R)
: A_(A), B_(B), Q_(Q), R_(R), K_(Eigen::MatrixXd::Zero(B.cols(), A.rows()))
{
  computeK();
}

void LqrController::computeK()
{
  // 初值取 Q（A 为稳定/含阻尼系统时可保证收敛）
  Eigen::MatrixXd P = Q_;

  // DARE 不动点迭代：
  //   P_{k+1} = A^T P_k A
  //             - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A + Q
  constexpr double kTol = 1e-12;
  constexpr int kMaxIter = 10000;
  for (int iter = 0; iter < kMaxIter; ++iter) {
    const Eigen::MatrixXd AtP = A_.transpose() * P;
    const Eigen::MatrixXd BtP = B_.transpose() * P;

    Eigen::MatrixXd P_next = Q_;
    P_next += AtP * A_;
    P_next -= AtP * B_ * (R_ + BtP * B_).inverse() * BtP * A_;

    const double diff = (P_next - P).norm();
    P = P_next;
    if (diff < kTol) {
      break;
    }
  }

  P_ = P;

  // 最优反馈增益：K = (R + B^T P B)^{-1} B^T P A
  const Eigen::MatrixXd BtP = B_.transpose() * P_;
  K_ = (R_ + BtP * B_).inverse() * BtP * A_;
}

double LqrController::computeForce(
  const Eigen::VectorXd & x,
  const Eigen::VectorXd & x_ref) const
{
  const Eigen::VectorXd e = x - x_ref;
  // 单输入：K 为 1 x n，控制量 u = -K * e（标量）
  return -(K_ * e)(0);
}

}  // namespace suspension_sim
