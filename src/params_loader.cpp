// Copyright 2026
// Apache-2.0

#include "suspension_sim/params_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "suspension_sim/quarter_car_model.hpp"

namespace suspension_sim
{

ModelParams loadParams(const std::string & yaml_file)
{
  ModelParams params;  // 先使用默认值

  std::ifstream fin(yaml_file);
  if (!fin.is_open()) {
    std::cerr << "[loadParams] 无法打开 YAML 文件: " << yaml_file
              << "，使用默认参数" << std::endl;
    return params;
  }

  try {
    const YAML::Node root = YAML::Load(fin);

    // 参数既可写在 model: 段下，也可直接写在顶层
    const YAML::Node model_node = root["model"] ? root["model"] : root;

    if (model_node["type"]) {params.type = model_node["type"].as<std::string>();}
    if (model_node["ms"]) {params.ms = model_node["ms"].as<double>();}
    if (model_node["mus"]) {params.mus = model_node["mus"].as<double>();}
    if (model_node["ks"]) {params.ks = model_node["ks"].as<double>();}
    if (model_node["cs"]) {params.cs = model_node["cs"].as<double>();}
    if (model_node["kt"]) {params.kt = model_node["kt"].as<double>();}
    // rate 允许写在 model 段，也可写在顶层（兼容两种布局）
    if (model_node["rate"]) {
      params.rate = model_node["rate"].as<double>();
    } else if (root["rate"]) {
      params.rate = root["rate"].as<double>();
    }

    std::cout << "[loadParams] 已从 " << yaml_file << " 加载参数: type="
              << params.type << " ms=" << params.ms << " mus=" << params.mus
              << " ks=" << params.ks << " cs=" << params.cs << " kt=" << params.kt
              << std::endl;
  } catch (const YAML::Exception & e) {
    std::cerr << "[loadParams] YAML 解析失败: " << e.what()
              << "，使用默认参数" << std::endl;
  }

  return params;
}

std::unique_ptr<SuspensionModel> createModel(const ModelParams & params)
{
  if (params.type == "quarter_car_2dof") {
    return std::make_unique<QuarterCarModel2DOF>(
      params.ms, params.mus, params.ks, params.cs, params.kt);
  }

  // 未知类型：回退到默认的二自由度四分之一车模型
  std::cerr << "[createModel] 未知模型类型 '" << params.type
            << "'，回退到 quarter_car_2dof" << std::endl;
  return std::make_unique<QuarterCarModel2DOF>(
    params.ms, params.mus, params.ks, params.cs, params.kt);
}

}  // namespace suspension_sim
