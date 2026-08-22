#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "suspension_sim/msg/suspension_state.hpp"
#include "suspension_sim/params_loader.hpp"
#include "suspension_sim/suspension_model.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief 悬架模型节点：订阅路面高度，更新悬架模型并发布状态
///
/// 订阅话题：road_height (std_msgs::msg::Float64)
/// 发布话题：suspension_state (suspension_sim::msg::SuspensionState)
/// 定时器以固定周期（默认 100 Hz / 10 ms）驱动模型积分一步。
///
/// 模型通过基类指针 std::unique_ptr<SuspensionModel> 持有：
/// 具体模型由 YAML 配置（config/model_params.yaml，参数 `model_yaml`）
/// 经 loadParams + createModel 工厂构造，也支持 ROS 参数覆盖。
class ModelNode : public rclcpp::Node
{
public:
  ModelNode()
  : Node("model_node"), start_time_(this->now())
  {
    // YAML 配置文件路径（优先级：启动参数 > 环境变量 > 编译期默认路径）
    this->declare_parameter<std::string>("model_yaml", "");
    std::string yaml_file = get_parameter("model_yaml").as_string();
    if (yaml_file.empty()) {
      const char * env = std::getenv("MODEL_YAML");
      yaml_file = (env != nullptr) ? env : MODEL_YAML_DEFAULT;
    }

    // 从 YAML 读取物理参数
    ModelParams params = loadParams(yaml_file);

    // ROS 参数可在 YAML 基础上继续覆盖（未显式设置的走 YAML 默认值）
    this->declare_parameter<double>("ms", params.ms);
    this->declare_parameter<double>("mus", params.mus);
    this->declare_parameter<double>("ks", params.ks);
    this->declare_parameter<double>("cs", params.cs);
    this->declare_parameter<double>("kt", params.kt);
    this->declare_parameter<double>("rate", params.rate);

    params.ms = get_parameter("ms").as_double();
    params.mus = get_parameter("mus").as_double();
    params.ks = get_parameter("ks").as_double();
    params.cs = get_parameter("cs").as_double();
    params.kt = get_parameter("kt").as_double();
    params.rate = get_parameter("rate").as_double();

    // 工厂构造具体模型，节点只持有基类指针
    model_ = createModel(params);

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
      "model_node started: model='%s' subscribing 'road_height', publishing "
      "'suspension_state' at %.1f Hz (yaml=%s)",
      model_->name().c_str(), get_parameter("rate").as_double(), yaml_file.c_str());
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
  std::unique_ptr<SuspensionModel> model_;  // 基类指针
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
