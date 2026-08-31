// Copyright 2026
// Apache-2.0

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include "suspension_sim/lqr_controller.hpp"
#include "suspension_sim/msg/suspension_state.hpp"
#include "suspension_sim/srv/estimate_state.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief LQR 控制器节点
///
/// 状态来源（由参数 `state_source` 选择）：
///   - "service"（默认）：作为客户端调用 estimator_node 的 estimate_state 服务
///     获取*估计状态*；服务不可用时回退为直接订阅 suspension_state（真值）
///   - "topic"：直接订阅 suspension_state（真值，旧行为；后期将改为订阅
///     估计话题 estimated_state）
/// 发布话题：control_force (std_msgs::msg::Float64)
///
/// 控制器使用标准悬架状态空间（平衡点为 0，x_ref = 0）：
///   z = [z1, z2, z3, z4]
///     z1 = xs - xus     悬架动行程 (m)
///     z2 = xus - r      轮胎变形 (m)，r 为路面高度
///     z3 = xs_dot       簧上质量速度 (m/s)
///     z4 = xus_dot      簧下质量速度 (m/s)
///
/// 连续系统矩阵（线性化于平衡点，r 为扰动输入，不进入 A/B）：
///   dz1/dt = z3 - z4
///   dz2/dt = z4
///   dz3/dt = (-ks*z1 - cs*(z3-z4) + u) / ms
///   dz4/dt = ( ks*z1 + cs*(z3-z4) - kt*z2 - u) / mus
///
/// 离散化采用零阶保持（ZOH，矩阵指数），再由 LqrController 求解
/// 离散 Riccati 方程得到最优增益 K，控制律：u = -K * z。
///
/// Q、R 的对角线元素通过 ROS 参数加载（参数名 lqr_q0..lqr_q3、lqr_r0..）。
/// x_ref 当前固定为 0（悬架平衡点），后续可扩展为参考跟踪。
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    // 默认 Q / R（可被 YAML 或命令行参数覆盖）
    // Q 对应 [z1(动行程), z2(轮胎变形), z3(簧上速度), z4(簧下速度)]
    declare_parameter("lqr_q0", 1e3);   // z1^2 加权（动行程）
    declare_parameter("lqr_q1", 1e3);   // z2^2 加权（轮胎变形）
    declare_parameter("lqr_q2", 1.0);   // z3^2 加权（簧上速度）
    declare_parameter("lqr_q3", 1.0);   // z4^2 加权（簧下速度）
    declare_parameter("lqr_r0", 1e-6);  // u^2 加权

    // 控制频率（与模型更新频率一致，用于离散化）
    declare_parameter("rate", 100.0);

    // 状态来源：service（调用估计服务）/ topic（直接订阅 suspension_state）
    declare_parameter<std::string>("state_source", "service");

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

    // 1) 组装 Q / R 对角阵
    Eigen::MatrixXd Q(4, 4);
    Q.setZero();
    Q(0, 0) = get_parameter("lqr_q0").as_double();
    Q(1, 1) = get_parameter("lqr_q1").as_double();
    Q(2, 2) = get_parameter("lqr_q2").as_double();
    Q(3, 3) = get_parameter("lqr_q3").as_double();

    Eigen::MatrixXd R(1, 1);
    R(0, 0) = get_parameter("lqr_r0").as_double();

    // 2) 连续系统矩阵（悬架坐标 z = [z1,z2,z3,z4]）
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
    B(2, 0) = 1.0 / ms;    // u 对 z3 的影响
    B(3, 0) = -1.0 / mus;  // u 对 z4 的影响

    // 3) 零阶保持（ZOH）离散化：Ad = exp(A*dt), Bd = integral(exp(A*t)B)
    //    通过增广矩阵指数一次性求得（比显式欧拉更精确稳定）。
    const int n = 4, m = 1;
    Eigen::MatrixXd M(n + m, n + m);
    M.setZero();
    M.topLeftCorner(n, n) = A * dt;
    M.topRightCorner(n, m) = B * dt;
    const Eigen::MatrixXd Md = M.exp();
    const Eigen::MatrixXd Ad = Md.topLeftCorner(n, n);
    const Eigen::MatrixXd Bd = Md.topRightCorner(n, m);

    // 4) 求解离散 Riccati 方程并计算最优增益 K
    controller_ = std::make_shared<LqrController>(Ad, Bd, Q, R);

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("control_force", 10);

    // 5) 选择状态来源
    const std::string state_source = get_parameter("state_source").as_string();
    if (state_source == "service") {
      // 作为服务客户端，向 estimator_node 请求估计状态（默认）
      client_ = this->create_client<suspension_sim::srv::EstimateState>(
        "estimate_state");
      while (!client_->wait_for_service(2s)) {
        RCLCPP_WARN(
          this->get_logger(),
          "estimate_state service not available, retrying...");
      }
      service_timer_ = this->create_wall_timer(
        10ms, std::bind(&ControllerNode::on_service_timer, this));
    } else {
      // 直接订阅状态（真值 / 旧行为）
      subscription_ = this->create_subscription<suspension_sim::msg::SuspensionState>(
        "suspension_state", 10,
        [this](const suspension_sim::msg::SuspensionState::SharedPtr msg) {
          on_state(msg->xs, msg->xus, msg->road_height, msg->xs_dot, msg->xus_dot);
        });
    }

    RCLCPP_INFO(
      this->get_logger(),
      "controller_node started: state_source='%s', publishing "
      "'control_force' (dt=%.4f s). K = [%.4f, %.4f, %.4f, %.4f]",
      state_source.c_str(), dt,
      controller_->K()(0, 0), controller_->K()(0, 1),
      controller_->K()(0, 2), controller_->K()(0, 3));
  }

private:
  void on_service_timer()
  {
    if (!client_->service_is_ready()) {
      return;
    }
    // 无在途请求时发起新请求；否则若已就绪则取响应并计算控制力
    if (!pending_future_.valid()) {
      request_ = std::make_shared<suspension_sim::srv::EstimateState::Request>();
      pending_future_ = client_->async_send_request(request_).future;
      return;
    }
    if (pending_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto resp = pending_future_.get();
      // 从绝对坐标构造悬架坐标 z（z_ref = 0）
      const double z1 = resp->xs - resp->xus;              // 悬架动行程估计
      const double z2 = resp->xus - resp->road_height;     // 轮胎变形估计
      Eigen::VectorXd z(4);
      z << z1, z2, resp->xs_dot, resp->xus_dot;

      const double force = controller_->computeForce(z, Eigen::VectorXd::Zero(4));

      auto out = std_msgs::msg::Float64();
      out.data = force;
      publisher_->publish(out);

      // 本轮请求处理完毕，发起下一次请求
      pending_future_ = client_->async_send_request(request_).future;
    }
  }

  void on_state(
    double xs, double xus, double road_height, double xs_dot, double xus_dot)
  {
    // 从绝对坐标构造悬架坐标 z（z_ref = 0）
    const double z1 = xs - xus;              // 悬架动行程
    const double z2 = xus - road_height;     // 轮胎变形
    Eigen::VectorXd z(4);
    z << z1, z2, xs_dot, xus_dot;

    const double force = controller_->computeForce(z, Eigen::VectorXd::Zero(4));

    auto out = std_msgs::msg::Float64();
    out.data = force;
    publisher_->publish(out);
  }

  rclcpp::Subscription<suspension_sim::msg::SuspensionState>::SharedPtr subscription_;
  rclcpp::Client<suspension_sim::srv::EstimateState>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::shared_ptr<LqrController> controller_;
  std::shared_ptr<suspension_sim::srv::EstimateState::Request> request_;
  std::future<suspension_sim::srv::EstimateState::Response::SharedPtr> pending_future_;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
