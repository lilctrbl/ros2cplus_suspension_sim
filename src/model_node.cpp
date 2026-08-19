#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "suspension_sim/quarter_car_model.hpp"
#include "suspension_sim/msg/suspension_state.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief 悬架模型节点：订阅路面高度，更新 2DOF 四分之一车模型并发布状态
///
/// 订阅话题：road_height (std_msgs::msg::Float64)
/// 发布话题：suspension_state (suspension_sim::msg::SuspensionState)
/// 定时器以固定周期（默认 100 Hz / 10 ms）驱动模型积分一步。
class ModelNode : public rclcpp::Node
{
public:
  ModelNode()
  : Node("model_node"), start_time_(this->now())
  {
    // 二自由度四分之一车模型参数（默认值，可通过参数覆盖）
    this->declare_parameter<double>("ms", 300.0);    // 簧上质量 kg
    this->declare_parameter<double>("mus", 40.0);    // 簧下质量 kg
    this->declare_parameter<double>("ks", 15000.0);  // 悬架刚度 N/m
    this->declare_parameter<double>("cs", 1000.0);   // 悬架阻尼 N·s/m
    this->declare_parameter<double>("kt", 200000.0); // 轮胎刚度 N/m
    this->declare_parameter<double>("rate", 100.0);  // 更新频率 Hz

    model_ = std::make_unique<QuarterCarModel2DOF>(
      get_parameter("ms").as_double(),
      get_parameter("mus").as_double(),
      get_parameter("ks").as_double(),
      get_parameter("cs").as_double(),
      get_parameter("kt").as_double());

    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "road_height",
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        latest_road_height_ = msg->data;
      });

    publisher_ = this->create_publisher<suspension_sim::msg::SuspensionState>(
      "suspension_state", 10);

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / get_parameter("rate").as_double()));
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&ModelNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "model_node started: subscribing 'road_height', publishing 'suspension_state' at %.1f Hz",
      get_parameter("rate").as_double());
  }

private:
  void on_timer()
  {
    const double dt = 1.0 / get_parameter("rate").as_double();
    model_->update(latest_road_height_, dt);

    auto msg = suspension_sim::msg::SuspensionState();
    msg.time = (this->now() - start_time_).seconds();
    msg.road_height = latest_road_height_;

    const auto & state = model_->getState();
    msg.xs = state(0);
    msg.xus = state(1);
    msg.xs_dot = state(2);
    msg.xus_dot = state(3);
    msg.body_accel = model_->getBodyAccel();

    publisher_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<suspension_sim::msg::SuspensionState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<QuarterCarModel2DOF> model_;
  const rclcpp::Time start_time_;
  double latest_road_height_ = 0.0;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::ModelNode>());
  rclcpp::shutdown();
  return 0;
}
