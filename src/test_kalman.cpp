// Copyright 2026
// Apache-2.0
//
// 独立测试程序（不依赖 rclcpp）：
//   1. 用四分之一车模型（绝对坐标，显式欧拉）在正弦路面上生成"真值"状态
//   2. 由真值构造含高斯测量噪声的观测 y = [z2(轮胎变形), z4(簧下速度)]
//   3. 构造 KalmanFilter（悬架坐标，ZOH 离散；已知输入为路面速度 r_dot，
//      由相邻两次 road_height 数值差分得到，B = [0,-1,0,0]^T）
//   4. 对比滤波估计 / 真值 / 含噪观测，输出均方根误差（RMSE）
//
// 用法：ros2 run suspension_sim test_kalman
//       （或直接 ./build/suspension_sim/test_kalman）

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <cstdio>
#include <random>

#include "suspension_sim/kalman_filter.hpp"

namespace
{

/// 标准正态随机数生成器（Box-Muller / C++11 <random>）
std::mt19937 g_rng(42u);
std::normal_distribution<double> g_gauss(0.0, 1.0);

double randn() {return g_gauss(g_rng);}

}  // namespace

int main()
{
  // ---- 物理参数（与 config/model_params.yaml 一致）----
  const double ms = 300.0;
  const double mus = 40.0;
  const double ks = 15000.0;
  const double cs = 1000.0;
  const double kt = 200000.0;
  const double dt = 0.01;  // 100 Hz

  // ---- 悬架坐标连续系统矩阵 z = [z1,z2,z3,z4] = [xs-xus, xus-r, xs_dot, xus_dot] ----
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

  // 输入矩阵 B：已知输入为路面速度 r_dot（悬架坐标中 dz2/dt = z4 - r_dot）
  Eigen::MatrixXd B(4, 1);
  B.setZero();
  B(1, 0) = -1.0;

  // ---- ZOH 离散化（增广矩阵指数）----
  Eigen::MatrixXd M(5, 5);
  M.setZero();
  M.topLeftCorner(4, 4) = A * dt;
  M.topRightCorner(4, 1) = B * dt;
  const Eigen::MatrixXd Md = M.exp();
  const Eigen::MatrixXd Ad = Md.topLeftCorner(4, 4);
  const Eigen::MatrixXd Bd = Md.topRightCorner(4, 1);

  // ---- 观测矩阵 C：观测 [z2(轮胎变形), z4(簧下速度)] ----
  Eigen::MatrixXd C(2, 4);
  C.setZero();
  C(0, 1) = 1.0;
  C(1, 3) = 1.0;

  // ---- 噪声协方差 ----
  const double sig_y = 1e-3;   // 测量噪声标准差（m 或 m/s）
  const double sig_w = 1e-4;   // 过程噪声标准差（覆盖路面输入等模型误差）
  Eigen::MatrixXd Q_proc = Eigen::MatrixXd::Identity(4, 4) * (sig_w * sig_w);
  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Identity(2, 2) * (sig_y * sig_y);

  // ---- 构造卡尔曼滤波器 ----
  suspension_sim::KalmanFilter kf(Ad, Bd, C, Q_proc, R_meas);

  // ---- 真值仿真：正弦路面，绝对坐标显式欧拉 ----
  const double amplitude = 0.05;   // m
  const double freq = 0.5;         // Hz
  constexpr int kSteps = 2000;     // 20 s

  double xs = 0.0, xus = 0.0, xs_dot = 0.0, xus_dot = 0.0;  // 真值（绝对坐标）
  double road = 0.0;

  double rmse_meas[2] = {0.0, 0.0};  // 含噪观测相对真值
  double rmse_est[4] = {0.0, 0.0, 0.0, 0.0};  // 滤波估计相对真值
  int count = 0;

  for (int i = 0; i < kSteps; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double road_next = amplitude * std::sin(2.0 * M_PI * freq * (t + dt));
    road = amplitude * std::sin(2.0 * M_PI * freq * t);

    // 真值一步演化（显式欧拉，路面激励）
    const double xs_ddot = (-ks * (xs - xus) - cs * (xs_dot - xus_dot)) / ms;
    const double xus_ddot =
      (ks * (xs - xus) + cs * (xs_dot - xus_dot) - kt * (xus - road)) / mus;
    xs_dot += xs_ddot * dt;
    xus_dot += xus_ddot * dt;
    xs += xs_dot * dt;
    xus += xus_dot * dt;

    // 真值（悬架坐标）与含噪观测
    const double z2_true = xus - road;   // 轮胎变形
    const double z4_true = xus_dot;      // 簧下速度

    Eigen::Vector2d v;
    v << randn() * sig_y, randn() * sig_y;
    Eigen::Vector2d y = Eigen::Vector2d(z2_true, z4_true) + v;

    // 已知输入：路面速度 r_dot（中心差分，与 estimator_node 一致）
    const double r_dot = (road_next - road) / dt;
    Eigen::VectorXd u(1);
    u << r_dot;

    // 卡尔曼一步（predict + update）
    kf.step(y, u);
    const Eigen::VectorXd z_est = kf.xHat();

    // 累计误差（跳过前 20 步，等待滤波器收敛）
    if (i >= 20) {
      rmse_meas[0] += (y(0) - z2_true) * (y(0) - z2_true);
      rmse_meas[1] += (y(1) - z4_true) * (y(1) - z4_true);
      rmse_est[0] += (z_est(0) - (xs - xus)) * (z_est(0) - (xs - xus));
      rmse_est[1] += (z_est(1) - z2_true) * (z_est(1) - z2_true);
      rmse_est[2] += (z_est(2) - xs_dot) * (z_est(2) - xs_dot);
      rmse_est[3] += (z_est(3) - z4_true) * (z_est(3) - z4_true);
      ++count;
    }
  }

  for (int k = 0; k < 2; ++k) {
    rmse_meas[k] = std::sqrt(rmse_meas[k] / count);
  }
  for (int k = 0; k < 4; ++k) {
    rmse_est[k] = std::sqrt(rmse_est[k] / count);
  }

  std::printf("===== 卡尔曼滤波测试（正弦路面 %.2f m / %.1f Hz，20 s）=====\n",
              amplitude, freq);
  std::printf("测量噪声 std = %.1e，过程噪声 std = %.1e\n", sig_y, sig_w);
  std::printf("\n状态           含噪观测 RMSE      滤波估计 RMSE\n");
  std::printf("z1 动行程  :        --           %.6f m\n", rmse_est[0]);
  std::printf("z2 轮胎变形:  %.6f m        %.6f m\n", rmse_meas[0], rmse_est[1]);
  std::printf("z3 簧上速度:        --           %.6f m/s\n", rmse_est[2]);
  std::printf("z4 簧下速度:  %.6f m/s      %.6f m/s\n", rmse_meas[1], rmse_est[3]);

  std::printf("\n（滤波估计 RMSE 应明显小于含噪观测 RMSE，说明滤波有效）\n");
  return 0;
}
