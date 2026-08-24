// Copyright 2026
// Apache-2.0
//
// 独立测试程序（不依赖 rclcpp）：
//   1. 由物理参数构造悬架坐标连续系统矩阵，ZOH 离散化（dt=10ms）
//   2. 构造 LqrController（内部求解离散 Riccati 方程并计算 K）
//   3. 初始条件响应仿真（悬架被压缩 5cm），对比开环 / 闭环衰减
//   4. 打印 K 与衰减性能对比
//
// 说明：采用离散状态方程 x_{k+1} = Ad*x_k + Bd*u_k 仿真，
// 避免显式欧拉在车身高频模态下的数值不稳定。
//
// 用法：ros2 run suspension_sim test_lqr
//       （或直接 ./build/suspension_sim/test_lqr）

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <cstdio>

#include "suspension_sim/lqr_controller.hpp"

int main()
{
  // ---- 物理参数（与 config/model_params.yaml 一致）----
  const double ms = 300.0;
  const double mus = 40.0;
  const double ks = 15000.0;
  const double cs = 1000.0;
  const double kt = 200000.0;
  const double dt = 0.01;  // 100 Hz

  // ---- 连续系统矩阵（悬架坐标 z = [z1,z2,z3,z4] = [xs-xus, xus-r, xs_dot, xus_dot]）----
  Eigen::MatrixXd A(4, 4);
  A.setZero();
  A(0, 2) = 1.0;
  A(0, 3) = -1.0;
  A(1, 3) = 1.0;
  A(2, 0) = -ks / ms;
  A(2, 2) = -cs / ms;
  A(2, 3) = cs / ms;
  A(3, 0) = ks / mus;
  A(3, 1) = -kt / mus;
  A(3, 2) = cs / mus;
  A(3, 3) = -cs / mus;

  Eigen::MatrixXd B(4, 1);
  B.setZero();
  B(2, 0) = 1.0 / ms;
  B(3, 0) = -1.0 / mus;

  // ---- 权重（与 controller_node 默认参数一致）----
  Eigen::MatrixXd Q(4, 4);
  Q.setZero();
  Q(0, 0) = 1e3;
  Q(1, 1) = 1e3;
  Q(2, 2) = 1.0;
  Q(3, 3) = 1.0;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = 1e-6;

  // ---- ZOH 离散化（增广矩阵指数）----
  Eigen::MatrixXd M(5, 5);
  M.setZero();
  M.topLeftCorner(4, 4) = A * dt;
  M.topRightCorner(4, 1) = B * dt;
  const Eigen::MatrixXd Md = M.exp();
  const Eigen::MatrixXd Ad = Md.topLeftCorner(4, 4);
  const Eigen::MatrixXd Bd = Md.topRightCorner(4, 1);

  // ---- 构造 LQR 控制器（构造函数内自动 computeK）----
  suspension_sim::LqrController lqr(Ad, Bd, Q, R);
  std::printf("K = [%10.2f %10.2f %10.2f %10.2f]\n",
              lqr.K()(0, 0), lqr.K()(0, 1), lqr.K()(0, 2), lqr.K()(0, 3));

  // ---- 初始条件响应：悬架被压缩 5 cm（z1 = 0.05），其余状态为 0 ----
  constexpr int kSteps = 1000;  // 10 s
  Eigen::Vector4d z_open(0.05, 0.0, 0.0, 0.0);
  Eigen::Vector4d z_closed(0.05, 0.0, 0.0, 0.0);
  Eigen::Vector4d z_ref = Eigen::Vector4d::Zero();

  double open_peak = 0.0, closed_peak = 0.0;
  int open_settle = 0, closed_settle = 0;  // 衰减到峰值 10% 所需步数
  bool open_settled = false, closed_settled = false;

  for (int i = 0; i < kSteps; ++i) {
    // 开环（u = 0）
    z_open = Ad * z_open;
    // 闭环（u = -K z）
    const double u = lqr.computeForce(z_closed, z_ref);
    z_closed = Ad * z_closed + Bd * (u);

    open_peak = std::max(open_peak, std::fabs(z_open(0)));
    closed_peak = std::max(closed_peak, std::fabs(z_closed(0)));

    // 记录衰减到峰值 10% 的步数
    if (!open_settled && std::fabs(z_open(0)) < 0.1 * open_peak) {
      open_settle = i;
      open_settled = true;
    }
    if (!closed_settled && std::fabs(z_closed(0)) < 0.1 * closed_peak) {
      closed_settle = i;
      closed_settled = true;
    }
  }

  std::printf("\n===== 初始条件响应对比（悬架压缩 5 cm，z1 = xs - xus）=====\n");
  std::printf("              开环(被动)     闭环(LQR)\n");
  std::printf("动行程峰值 z1 : %10.5f m  %10.5f m\n", open_peak, closed_peak);
  std::printf("衰减到 10%%   : %10.3f s  %10.3f s\n",
              open_settled ? open_settle * dt : -1.0,
              closed_settled ? closed_settle * dt : -1.0);
  std::printf("10s 末值 z1  : %10.6f m  %10.6f m\n", z_open(0), z_closed(0));

  return 0;
}
