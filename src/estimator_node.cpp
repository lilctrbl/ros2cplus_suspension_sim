// Copyright 2026
// Apache-2.0

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include "suspension_sim/kalman_filter.hpp"
#include "suspension_sim/msg/suspension_state.hpp"
#include "suspension_sim/srv/estimate_state.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief 状态估计节点：卡尔曼滤波融合仿真状态（含噪声），并提供估计服务
///
/// 订阅话题：suspension_state (suspension_sim::msg::SuspensionState，真值+噪声)
/// 提供服务：estimate_state (suspension_sim::srv::EstimateState，返回当前估计)
///
/// 采用与 LQR 控制器一致的*悬架状态空间*（平衡点 0）：
///   z = [z1, z2, z3, z4]
///     z1 = xs - xus     悬架动行程 (m)
///     z2 = xus - r      轮胎变形 (m)，r 为路面高度
///     z3 = xs_dot       簧上质量速度 (m/s)
///     z4 = xus_dot      簧下质量速度 (m/s)
///
/// 连续系统矩阵与 controller_node 相同；已知输入为路面速度 r_dot
/// （悬架坐标中 dz2/dt = z4 - r_dot），由相邻两次 road_height 数值差分得到，
/// 经 B = [0,-1,0,0]^T 进入预测步。观测取 z2（轮胎变形）与 z4（簧下
/// 速度），均含零均值高斯测量噪声，由观测噪声协方差 R_meas 表达。
/// 过程噪声协方差 Q_proc 表达模型误差 / 未建模的控制力 / 路面扰动。
///
/// 每次收到状态消息：
///   1. 由测量构造观测 y = [z2, z4]（测量含噪声）
///   2. ZOH 离散系统矩阵经 KalmanFilter 做一步 predict + update
///   3. 服务请求时返回当前滤波状态（均值 x_hat）
class EstimatorNode : public rclcpp::Node
{
public:
  EstimatorNode()
  : Node("estimator_node"), start_time_(this->now())
  {
    // 噪声 / 滤波参数（可经 ROS 参数覆盖）
    declare_parameter("measurement_noise", 1e-4);   // 观测噪声标准差（位移/速度，m 或 m/s）
    declare_parameter("process_noise", 1e-6);       // 过程噪声标准差
    declare_parameter("rate", 100.0);               // 模型更新频率（与 model_node 一致，用于离散化）

    // 物理参数（与 config/model_params.yaml 保持一致）
    declare_parameter("ms", 300.0);
    declare_parameter("mus", 40.0);
    declare_parameter("ks", 15000.0);
    declare_parameter("cs", 1000.0);
    declare_parameter("kt", 200000.0);

    const double ms = get_parameter("ms").as_double();
    const double mus = get_parameter("mus").as_double();
    const double ks = get_parameter("ks").as_double();
    const double cs = get_parameter("cs").as_double();
    const double kt = get_parameter("kt").as_double();
    const double dt = 1.0 / get_parameter("rate").as_double();
    const double sig_y = get_parameter("measurement_noise").as_double();
    const double sig_w = get_parameter("process_noise").as_double();

    // 1) 连续系统矩阵（悬架坐标 z = [z1,z2,z3,z4]）
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

    // 2) 输入矩阵 B：已知输入为路面速度 r_dot（悬架坐标中 dz2/dt = z4 - r_dot）
    //    控制力 u 未直接建模（视为 0，误差由过程噪声覆盖）
    Eigen::MatrixXd B(4, 1);
    B.setZero();
    B(1, 0) = -1.0;

    // 3) 零阶保持（ZOH）离散化（与 controller_node 相同）
    Eigen::MatrixXd M(5, 5);
    M.setZero();
    M.topLeftCorner(4, 4) = A * dt;
    M.topRightCorner(4, 1) = B * dt;
    const Eigen::MatrixXd Md = M.exp();
    const Eigen::MatrixXd Ad = Md.topLeftCorner(4, 4);
    const Eigen::MatrixXd Bd = Md.topRightCorner(4, 1);

    // 4) 观测矩阵 C：观测 [z2(轮胎变形), z4(簧下速度)]
    Eigen::MatrixXd C(2, 4);
    C.setZero();
    C(0, 1) = 1.0;
    C(1, 3) = 1.0;

    // 5) 噪声协方差（对角线为方差 = 标准差^2）
    Eigen::MatrixXd Q_proc = Eigen::MatrixXd::Identity(4, 4) * (sig_w * sig_w);
    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Identity(2, 2) * (sig_y * sig_y);

    filter_ = std::make_shared<KalmanFilter>(Ad, Bd, C, Q_proc, R_meas);

    // 6) 订阅（含测量噪声的）模型状态
    subscription_ = this->create_subscription<suspension_sim::msg::SuspensionState>(
      "suspension_state", 10,
      [this](const suspension_sim::msg::SuspensionState::SharedPtr msg) {
        on_state(msg);
      });

    // 7) 估计服务：请求为空，返回当前状态估计
    service_ = this->create_service<suspension_sim::srv::EstimateState>(
      "estimate_state",
      [this](
        const suspension_sim::srv::EstimateState::Request::SharedPtr /*req*/,
        suspension_sim::srv::EstimateState::Response::SharedPtr resp) {
        on_estimate_service(resp);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "estimator_node started: subscribing 'suspension_state', serving "
      "'estimate_state' (dt=%.4f s, sig_y=%.1e, sig_w=%.1e)",
      dt, sig_y, sig_w);
  }

private:
  void on_state(const suspension_sim::msg::SuspensionState::SharedPtr msg)
  {
    // 悬架坐标（与 controller_node 相同的变换）：z2 = 轮胎变形
    const double z2 = msg->xus - msg->road_height;   // 轮胎变形

    // 路面速度 r_dot：由相邻两次 road_height 数值差分估计
    double r_dot = 0.0;
    if (has_measurement_) {
      const double dt = 1.0 / get_parameter("rate").as_double();
      r_dot = (msg->road_height - last_road_height_) / dt;
    }
    last_road_height_ = msg->road_height;

    // 观测 = [z2(轮胎变形), z4(簧下速度)] + 测量噪声；
    // 已知输入 u = [r_dot]（B 第 0 列）
    Eigen::Vector2d y;
    y << z2, msg->xus_dot;

    // 已知输入 u = [r_dot]（B 第 0 列）
    Eigen::VectorXd u(1);
    u << r_dot;
    filter_->step(y, u);

    // 记录最近一次测量对应的路面高度与时间（恢复绝对量 / 响应使用）
    latest_road_height_ = msg->road_height;
    last_time_ = (this->now() - start_time_).seconds();
    has_measurement_ = true;
    has_estimate_ = true;
  }

  void on_estimate_service(suspension_sim::srv::EstimateState::Response::SharedPtr resp)
  {
    const Eigen::VectorXd x = filter_->xHat();
    const double z1 = x(0);  // 悬架动行程估计
    const double z2 = x(1);  // 轮胎变形估计
    const double z3 = x(2);  // 簧上速度估计
    const double z4 = x(3);  // 簧下速度估计

    resp->time = has_estimate_ ? last_time_ : 0.0;
    resp->road_height = latest_road_height_;     // 路面高度真值（来自 model_node）
    resp->xs = z1 + z2 + latest_road_height_;    // xs = z1 + z2 + r
    resp->xus = z2 + latest_road_height_;        // xus = z2 + r
    resp->xs_dot = z3;
    resp->xus_dot = z4;
    resp->body_accel = 0.0;                      // 状态不含加速度，暂填 0

    // 记录到日志，便于对比估计 / 真值（z1 与 z2 均已滤波）
    RCLCPP_DEBUG(
      this->get_logger(),
      "estimate: z1=%.5f z2=%.5f z3=%.5f z4=%.5f (from service)",
      z1, z2, z3, z4);
  }

  rclcpp::Subscription<suspension_sim::msg::SuspensionState>::SharedPtr subscription_;
  rclcpp::Service<suspension_sim::srv::EstimateState>::SharedPtr service_;
  std::shared_ptr<KalmanFilter> filter_;
  const rclcpp::Time start_time_;
  double latest_road_height_ = 0.0;  // 最近一次测量中的路面高度（用于恢复绝对量）
  double last_road_height_ = 0.0;    // 上一次测量的路面高度（用于差分求 r_dot）
  double last_time_ = 0.0;
  bool has_measurement_ = false;
  bool has_estimate_ = false;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
