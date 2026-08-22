// Copyright 2026
// Apache-2.0

#ifndef SUSPENSION_SIM__PARAMS_LOADER_HPP_
#define SUSPENSION_SIM__PARAMS_LOADER_HPP_

#include <memory>
#include <string>

#include "suspension_sim/suspension_model.hpp"

namespace suspension_sim
{

/// @brief 悬架模型物理参数（与 YAML 中 model 段对应）
struct ModelParams
{
  std::string type = "quarter_car_2dof";  // 模型类型（工厂选择依据）
  double ms = 300.0;      // 簧上质量 (kg)
  double mus = 40.0;      // 簧下质量 (kg)
  double ks = 15000.0;    // 悬架刚度 (N/m)
  double cs = 1000.0;     // 悬架阻尼 (N·s/m)
  double kt = 200000.0;   // 轮胎刚度 (N/m)
  double rate = 100.0;    // 模型更新频率 (Hz)
};

/// @brief 从 YAML 文件读取模型参数
///
/// 参数既可写在顶层，也可写在 `model:` 段下（见 config/model_params.yaml）。
/// 文件不存在、字段缺失或解析失败时使用默认值，并打印警告。
/// @param yaml_file YAML 文件路径
/// @return 填充完成的 ModelParams
ModelParams loadParams(const std::string & yaml_file);

/// @brief 根据参数构造具体的悬架模型（工厂）
///
/// 目前支持 `quarter_car_2dof`；未知类型回退到默认模型并打印警告。
/// @param params 模型参数
/// @return 指向 SuspensionModel 的 unique_ptr
std::unique_ptr<SuspensionModel> createModel(const ModelParams & params);

}  // namespace suspension_sim

#endif  // SUSPENSION_SIM__PARAMS_LOADER_HPP_
