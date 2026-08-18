#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace suspension_sim
{

/// @brief 显示节点：订阅路面高度并打印
///
/// 订阅话题：road_height (std_msgs::msg::Float64)
/// 每收到一条消息就打印高度值，用于观察发布-订阅通信。
class RoadDisplayNode : public rclcpp::Node
{
public:
  RoadDisplayNode() : Node("road_display_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "road_height",
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "road_height = %.4f m", msg->data);
      });

    RCLCPP_INFO(this->get_logger(), "road_display_node started: subscribing 'road_height'");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::RoadDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
