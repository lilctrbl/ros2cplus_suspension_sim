// Copyright 2026
// Apache-2.0
//
// 独立测试程序（不依赖 rclcpp）：
//   1. 从 YAML 文件加载物理参数（loadParams）
//   2. 通过工厂 createModel 构造具体模型（基类指针）
//   3. 以正弦路面输入仿真若干步
//   4. 周期性打印模型状态

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "suspension_sim/params_loader.hpp"
#include "suspension_sim/suspension_model.hpp"

// 编译期由 CMake 注入默认 YAML 路径（target_compile_definitions: MODEL_YAML）。
// 这里提供兜底默认值：避免 IDE 静态分析（未配置 CMake 宏）误报未定义，
// 也保证脱离 CMake 直接编译时仍有可用默认值。
#ifndef MODEL_YAML
#define MODEL_YAML "config/model_params.yaml"
#endif

int main(int argc, char ** argv)
{
  // 默认使用编译期注入的 YAML 路径（CMake 中定义），也可用命令行参数覆盖
  std::string yaml_file = MODEL_YAML;
  if (argc > 1) {
    yaml_file = argv[1];
  }

  // 1. 加载参数
  const auto params = suspension_sim::loadParams(yaml_file);

  // 2. 工厂构造具体模型（持有基类指针）
  std::unique_ptr<suspension_sim::SuspensionModel> model =
    suspension_sim::createModel(params);

  std::printf("模型类型   : %s（状态维数 %zu）\n",
              model->name().c_str(), model->stateSize());
  std::printf("物理参数   : ms=%.1f mus=%.1f ks=%.1f cs=%.1f kt=%.1f\n",
              params.ms, params.mus, params.ks, params.cs, params.kt);

  // 3. 正弦路面输入，显式欧拉积分仿真 5 秒
  constexpr double kDt = 0.01;         // 100 Hz
  constexpr double kAmplitude = 0.05;  // m
  constexpr double kFrequency = 0.5;   // Hz
  constexpr int kSteps = 500;          // 共 5 s

  model->reset();
  std::printf("t(s)  road(m)  xs(m)     xus(m)    xs_dot    xus_dot   accel(m/s^2)\n");
  for (int i = 0; i <= kSteps; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const double road = kAmplitude * std::sin(2.0 * M_PI * kFrequency * t);
    model->update(road, kDt);

    // 每 100 步（1 s）及最后一步打印一次
    if (i % 100 == 0 || i == kSteps) {
      const auto state = model->getState();
      std::printf("%6.2f  %6.4f  %8.4f  %8.4f  %8.4f  %8.4f  %10.4f\n",
                  t, road, state(0), state(1), state(2), state(3),
                  model->getBodyAccel());
    }
  }

  return 0;
}
