# 版本历史

## v1.1（当前版本）

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
