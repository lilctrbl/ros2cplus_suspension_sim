# 版本历史

## v1.2（当前版本）

发布日期：2026-08-19

### 新增功能

- **二自由度四分之一车悬架模型**
  - 新增 `include/suspension_sim/quarter_car_model.hpp`，定义 `QuarterCarModel2DOF` 类
  - 状态向量 `[xs, xus, xs_dot, xus_dot]`，采用**显式欧拉积分**更新
  - 提供 `reset()` / `update()` / `getState()` / `getBodyAccel()` 接口
- **自定义消息 `SuspensionState.msg`**
  - 新增 `msg/SuspensionState.msg`，字段：`time`、`road_height`、`xs`、`xus`、`xs_dot`、`xus_dot`、`body_accel`
  - 通过 `rosidl_generate_interfaces` 生成，后续可扩展更多字段（如悬架动行程、轮胎载荷）
- **模型节点 `model_node`**
  - 新增 `src/model_node.cpp`：订阅 `road_height`，定时器回调中更新模型并发布 `suspension_state`（默认 100 Hz）
  - 模型参数（`ms`/`mus`/`ks`/`cs`/`kt`/`rate`）可通过 ROS 参数覆盖

### 配置变更

- `CMakeLists.txt`：新增 `rosidl_default_generators` 依赖、`rosidl_generate_interfaces` 生成自定义消息、新增 `model_node` 可执行目标
- `package.xml`：新增 `rosidl_default_generators` / `rosidl_default_runtime` 依赖及 `rosidl_interface_packages` 组

### 文档更新

- `RUNNING.md`：补充模型节点运行步骤与自定义消息查看方式
- `learning_notes.txt`：新增本次掌握的四个知识点（自定义消息、欧拉积分、定时器+订阅缓存模式、Eigen 库）

---

## v1.1

发布日期：2026-08-16

### 新增功能

- **多种路面模型（策略模式）**
  - 新增 `include/suspension_sim/road_profile.hpp`，定义抽象基类 `RoadProfile` 及 4 种实现：
    - `SineRoadProfile`：正弦路面 `y = A·sin(2π·f·t)`
    - `SquareRoadProfile`：方波路面（±A 跳变）
    - `RandomRoadProfile`：随机路面（按采样间隔随机跳变）
    - `SpeedBumpRoadProfile`：减速带路面（单次三角凸起）
- **路面热切换**
  - 新增 `road_profile` / `amplitude` / `frequency` 参数，支持运行时通过 `ros2 param set` 切换路面，无需重启节点
- **新增 `rcl_interfaces` 依赖**（参数设置回调返回值所需）

### 重构

- `road_input_node.cpp` 由固定正弦路面重构为策略模式，节点仅持有一个 `RoadProfile` 指针

### 变更说明

- 默认路面仍为正弦（`sine`），振幅 0.05 m、频率 0.5 Hz，与原 v1.0 行为一致

---

## v1.0

发布日期：2026-08-16

### 初始功能

- 新增 `src/road_input_node.cpp` 路面输入节点
- 以 **100 Hz** 发布话题 `road_height`（`std_msgs/msg/Float64`）
- 固定正弦路面：`y = 0.05 * sin(2 * PI * 0.5 * t)`
- 配置 `CMakeLists.txt` 与 `package.xml`，构建并通过运行验证（实测发布频率约 100 Hz）
