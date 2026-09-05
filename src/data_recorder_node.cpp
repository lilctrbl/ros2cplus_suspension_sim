// Copyright 2026
// Apache-2.0

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "suspension_sim/msg/suspension_state.hpp"

using namespace std::chrono_literals;

namespace suspension_sim
{

/// @brief 数据记录节点：订阅感兴趣话题并按固定周期写入 CSV 文件
///
/// 订阅话题：
///   - suspension_state (suspension_sim::msg::SuspensionState) 真值状态
///   - control_force   (std_msgs::msg::Float64)                控制力
///   - estimated_state (suspension_sim::msg::SuspensionState)  估计状态
///   - road_height     (std_msgs::msg::Float64)                路面高度
///
/// 定时器以固定周期（参数 rate，默认 100 Hz）采样，把各话题*最近一次*
/// 收到的值写入 CSV 一行。CSV 首列为带时间戳的同步采样时刻，其余列对应当前
/// 已订阅话题的字段；尚未收到过某话题时该列为空（写 CSV 时记为 NaN）。
///
/// CSV 格式（表头 + 数据行，UTF-8，浮点默认 %.6g）：
///   t, road_height, xs, xus, xs_dot, xus_dot, body_accel,
///   control_force, xs_est, xus_est
///
/// 写入采用"行缓冲 + 立即 flush"（flush 后即使 Ctrl+C 数据也已落盘），
/// 节点销毁（析构函数）时关闭文件流。
class DataRecorderNode : public rclcpp::Node
{
public:
  DataRecorderNode()
  : Node("data_recorder_node"), start_time_(this->now())
  {
    // 输出 CSV 路径（默认放到工作空间 logs/ 下；可 -p out_file:=xx 覆盖）
    this->declare_parameter<std::string>("out_file", "logs/suspension_log.csv");
    // 采样频率（Hz）：与模型发布频率保持一致时每拍都采样到新值
    this->declare_parameter<double>("rate", 100.0);

    const std::string out_file = get_parameter("out_file").as_string();
    const double rate = get_parameter("rate").as_double();
    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate));

    // 订阅话题
    state_sub_ = this->create_subscription<suspension_sim::msg::SuspensionState>(
      "suspension_state", 10,
      [this](const suspension_sim::msg::SuspensionState::SharedPtr msg) {
        rec_.xs = msg->xs;
        rec_.xus = msg->xus;
        rec_.xs_dot = msg->xs_dot;
        rec_.xus_dot = msg->xus_dot;
        rec_.body_accel = msg->body_accel;
        rec_.road_height = msg->road_height;
        rec_.has_state = true;
      });
    force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "control_force", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        rec_.control_force = msg->data;
        rec_.has_force = true;
      });
    est_sub_ = this->create_subscription<suspension_sim::msg::SuspensionState>(
      "estimated_state", 10,
      [this](const suspension_sim::msg::SuspensionState::SharedPtr msg) {
        rec_.xs_est = msg->xs;
        rec_.xus_est = msg->xus;
        rec_.has_est = true;
      });
    road_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "road_height", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        rec_.road_height = msg->data;
        rec_.has_road = true;
      });

    // 打开 CSV 并写入表头
    if (!open_file(out_file)) {
      // 打开失败：打印错误但保持节点运行（后续行写不进去），便于用户排查路径
      RCLCPP_ERROR(
        this->get_logger(), "failed to open '%s': %s",
        out_file.c_str(), std::strerror(errno));
    }

    timer_ = this->create_wall_timer(
      period_ms, std::bind(&DataRecorderNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "data_recorder_node started: subscribing suspension_state / "
      "control_force / estimated_state / road_height, writing CSV to '%s' "
      "at %.1f Hz", out_file.c_str(), rate);
  }

  ~DataRecorderNode() override
  {
    flush_and_close();
  }

private:
  /// 一次同步采样的全部待记录字段（含"是否已收到"标志）
  struct Record
  {
    bool has_state = false;
    bool has_road = false;
    bool has_force = false;
    bool has_est = false;
    double road_height = 0.0;
    double xs = 0.0;
    double xus = 0.0;
    double xs_dot = 0.0;
    double xus_dot = 0.0;
    double body_accel = 0.0;
    double control_force = 0.0;
    double xs_est = 0.0;
    double xus_est = 0.0;
  };

  /// 打开 CSV 文件并写表头；成功返回 true
  bool open_file(const std::string & path)
  {
    // 确保所在目录存在（默认 logs/）
    const auto slash = path.find_last_of('/');
    if (slash != std::string::npos) {
      const std::string dir = path.substr(0, slash);
      std::string cmd = "mkdir -p '" + dir + "'";
      if (std::system(cmd.c_str()) != 0) {
        RCLCPP_WARN(this->get_logger(), "could not create directory '%s'", dir.c_str());
      }
    }

    ofs_.open(path, std::ios::out | std::ios::trunc);
    if (!ofs_.is_open()) {
      return false;
    }
    ofs_ << "t,road_height,xs,xus,xs_dot,xus_dot,body_accel,"
      "control_force,xs_est,xus_est\n";
    ofs_.flush();
    return true;
  }

  void on_timer()
  {
    // 采样时刻 = 相对节点启动时间（与 SuspensionState.time 同源，便于对齐）
    const double t = (this->now() - start_time_).seconds();

    // 尚未打开文件（首拍前 open_file 失败过）则不再尝试
    if (!ofs_.is_open()) {
      return;
    }
    // 尚未收到任何话题 → 空行没意义，跳过（等消息到达）
    if (!rec_.has_state && !rec_.has_road && !rec_.has_force && !rec_.has_est) {
      return;
    }

    char buf[256];
    std::snprintf(
      buf, sizeof(buf),
      "%.6f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n",
      t,
      rec_.has_road ? rec_.road_height : std::nan(""),
      rec_.has_state ? rec_.xs : std::nan(""),
      rec_.has_state ? rec_.xus : std::nan(""),
      rec_.has_state ? rec_.xs_dot : std::nan(""),
      rec_.has_state ? rec_.xus_dot : std::nan(""),
      rec_.has_state ? rec_.body_accel : std::nan(""),
      rec_.has_force ? rec_.control_force : std::nan(""),
      rec_.has_est ? rec_.xs_est : std::nan(""),
      rec_.has_est ? rec_.xus_est : std::nan(""));
    ofs_ << buf;
    ofs_.flush();  // 每行落盘，Ctrl+C 也不丢数据
  }

  void flush_and_close()
  {
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
      RCLCPP_INFO(this->get_logger(), "data recorder closed CSV file");
    }
  }

  rclcpp::Subscription<suspension_sim::msg::SuspensionState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_sub_;
  rclcpp::Subscription<suspension_sim::msg::SuspensionState>::SharedPtr est_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr road_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  const rclcpp::Time start_time_;
  std::ofstream ofs_;
  Record rec_;
};

}  // namespace suspension_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<suspension_sim::DataRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
