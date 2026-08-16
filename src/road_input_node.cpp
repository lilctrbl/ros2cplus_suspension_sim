#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "suspension_sim/road_profile.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief 路面输入节点：以 100 Hz 发布路面高度
///
/// 发布话题：road_height (std_msgs::msg::Float64)
/// 路面模型通过 road_profile 参数选择，支持多种路面（策略模式）：
///   - sine       正弦路面   y = A * sin(2 * PI * f * t)
///   - square     方波路面
///   - random     随机路面
///   - speed_bump 减速带路面
/// 运行时可使用 `ros2 param set /road_input_node road_profile square`
/// 等命令热切换路面类型（通过参数回调重建实现）。
class RoadInputNode : public rclcpp::Node
{
public:
  RoadInputNode()
  : Node("road_input_node"), start_time_(this->now())
  {
    // 声明参数并提供默认值
    this->declare_parameter<std::string>("road_profile", "sine");
    this->declare_parameter<double>("amplitude", 0.05);
    this->declare_parameter<double>("frequency", 0.5);

    // 参数变化时重建路面实现（支持运行时热切换）
    // 注意：on_set_parameters_callback 在参数真正更新之前调用，因此
    // 必须使用传入的 params（即将生效的新值），不能在此处 get_parameter。
    param_callback_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        if (this->rebuild_profile(params)) {
          RCLCPP_INFO(
            this->get_logger(), "Rebuilt road profile -> '%s'",
            this->road_profile_->name().c_str());
        }
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        return result;
      });

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("road_height", 10);

    // 100 Hz -> 10 ms 周期
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RoadInputNode::on_timer, this));

    // 依据参数构建初始路面
    rebuild_profile();
    RCLCPP_INFO(
      this->get_logger(),
      "road_input_node started: publishing 'road_height' at 100 Hz, profile='%s'",
      road_profile_->name().c_str());
  }

private:
  void on_timer()
  {
    const double t = (this->now() - start_time_).seconds();

    auto msg = std_msgs::msg::Float64();
    msg.data = road_profile_->compute_height(t);

    publisher_->publish(msg);
  }

  /// 根据当前参数重建路面实现
  void rebuild_profile()
  {
    const auto type = get_parameter("road_profile").as_string();
    const double amp = get_parameter("amplitude").as_double();
    const double freq = get_parameter("frequency").as_double();
    rebuild_profile_impl(type, amp, freq);
  }

  /// 根据即将生效的新参数重建路面实现；返回 true 表示参数有变更
  bool rebuild_profile(const std::vector<rclcpp::Parameter> & params)
  {
    // 先从当前参数取值，再用传入的新参数覆盖，保证未提及的参数保持不变
    auto type = get_parameter("road_profile").as_string();
    auto amp = get_parameter("amplitude").as_double();
    auto freq = get_parameter("frequency").as_double();
    bool changed = false;

    for (const auto & p : params) {
      if (p.get_name() == "road_profile") {
        type = p.as_string();
        changed = true;
      } else if (p.get_name() == "amplitude") {
        amp = p.as_double();
        changed = true;
      } else if (p.get_name() == "frequency") {
        freq = p.as_double();
        changed = true;
      }
    }

    if (changed) {
      rebuild_profile_impl(type, amp, freq);
    }
    return changed;
  }

  /// 依据参数创建具体路面实现
  void rebuild_profile_impl(const std::string & type, double amp, double freq)
  {
    if (type == "square") {
      road_profile_ = std::make_shared<SquareRoadProfile>(amp, freq);
    } else if (type == "random") {
      road_profile_ = std::make_shared<RandomRoadProfile>(amp, freq);
    } else if (type == "speed_bump") {
      road_profile_ = std::make_shared<SpeedBumpRoadProfile>(amp, 2.0 / freq);
    } else {  // 默认 sine
      road_profile_ = std::make_shared<SineRoadProfile>(amp, freq);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;
  std::shared_ptr<RoadProfile> road_profile_;
  const rclcpp::Time start_time_;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::RoadInputNode>());
  rclcpp::shutdown();
  return 0;
}
