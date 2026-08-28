# 版本历史

## v2.2（当前版本）

发布日期：2026-08-28

### 新增功能

- **卡尔曼滤波器 `KalmanFilter`**
  - 新增 `include/suspension_sim/kalman_filter.hpp` + `src/kalman_filter.cpp`
  - 成员：`Eigen::MatrixXd A_, B_, C_, Q_proc_, R_meas_`、`Eigen::VectorXd x_hat_`、`Eigen::MatrixXd P_`
  - `predict(u)`：预测步 $\hat x \leftarrow A\hat x + Bu$、$P \leftarrow APA^T + Q_{proc}$
  - `update(y)`：更新步，计算卡尔曼增益 $K = P^-C^T(CP^-C^T + R_{meas})^{-1}$，修正 $\hat x$ 与 $P$（Joseph 形式，数值更稳定）
  - `step(y, u)`：一步预测 + 一步更新；`reset(x0, P0)` / `reset()` 重置状态
  - 构造函数自动做维度一致性检查
- **状态估计服务 `EstimateState.srv`**
  - 新增 `srv/EstimateState.srv`：请求为空，响应为悬架系统状态估计值（`time`/`road_height`/`xs`/`xus`/`xs_dot`/`xus_dot`/`body_accel`）
  - 经 `rosidl_generate_interfaces` 生成，服务名 `estimate_state`
- **状态估计节点 `estimator_node`**
  - 新增 `src/estimator_node.cpp`：订阅 `suspension_state`（含测量噪声），内部运行卡尔曼滤波
  - 提供 `estimate_state` 服务，返回当前滤波状态均值
  - 与 LQR 控制器共用悬架状态空间 $z=[x_s-x_{us}, x_{us}-r, \dot x_s, \dot x_{us}]$，ZOH 离散化
  - 观测为 $z_2$（轮胎变形）与 $z_4$（簧下速度），噪声协方差由 ROS 参数 `measurement_noise` / `process_noise` 配置
- **独立测试程序 `test_kalman`**
  - 新增 `src/test_kalman.cpp`，不依赖 rclcpp：构造含噪悬架系统仿真 → 卡尔曼滤波 → 对比含噪观测 / 滤波估计的 RMSE

### 配置变更

- `CMakeLists.txt`：新增 `srv/EstimateState.srv` 到 `rosidl_generate_interfaces`；新增 `estimator_node`、`test_kalman` 目标与 `kalman_filter.cpp` 源文件
- `package.xml`：版本号升至 `2.2.0`
- `.gitignore`：整理为分类清晰的完整版（含 rosbag2 录制数据忽略）

### 文档更新

- `README.md`：新增卡尔曼滤波原理、`KalmanFilter` 类说明、`estimator_node` 运行与服务调用方式、`test_kalman`、节点表与项目结构更新
- `RUNNING.md`：新增 `estimator_node` 手动运行步骤与服务调用示例
- `version.md`：新增本版本记录

### 本次新掌握知识点

- 卡尔曼滤波标准五式（预测 / 更新两步递归）；Joseph 形式协方差更新比 $P=(I-KC)P^-$ 数值更稳定
- ROS 2 服务（service）定义：`srv/*.srv`（`---` 分隔请求 / 响应），与消息一样由 `rosidl_generate_interfaces` 生成
- 服务调用：`ros2 service call /节点名/服务名 包/类型 "{}"`；`ros2 interface show` 查看接口定义
- 悬架坐标与绝对坐标的换算：$z_1+z_2 = x_s - r$、$z_2 = x_{us} - r$，恢复绝对量需借助路面高度 $r$

---

## v2.1

发布日期：2026-08-24

### 新增功能

- **LQR 控制器 `LqrController`**
  - 新增 `include/suspension_sim/lqr_controller.hpp` + `src/lqr_controller.cpp`
  - 成员：`Eigen::MatrixXd A_, B_, K_, Q_, R_`，构造函数传入 A/B/Q/R 并自动调用 `computeK()`
  - `computeK()`：求解**离散代数 Riccati 方程（DARE）**（不动点迭代），计算最优状态反馈增益 $K = (R + B^T P B)^{-1} B^T P A$
  - 控制律：`computeForce(x, x_ref)` 实现 $u = -K(x - x_{ref})$（`x_ref` 默认 0）
  - 说明：Eigen 本身不提供 Riccati 求解器（`care()` 是 MATLAB/SciPy 的接口且针对连续方程），故采用 DARE 不动点迭代
- **LQR 控制器节点 `controller_node`**
  - 新增 `src/controller_node.cpp`：订阅 `suspension_state`，发布 `control_force`（`std_msgs::msg::Float64`）
  - Q、R 对角线元素通过 ROS 参数加载：`lqr_q0..lqr_q3`（状态加权）、`lqr_r0`（输入加权）
  - 使用**标准悬架状态空间**：$z = [x_s-x_{us},\ x_{us}-r,\ \dot x_s,\ \dot x_{us}]$（动行程、轮胎变形、速度）
  - 采用**零阶保持（ZOH）离散化**（增广矩阵指数 `exp(A·dt)`）
  - 控制律：$u = -K \cdot z$，$x_{ref}=0$（悬架平衡点），后续可扩展为参考跟踪
- **模型接入主动力，实现闭环**
  - `SuspensionModel::update()` 增加 `control_force` 参数（默认 0）
  - `QuarterCarModel2DOF` 动力学加入主动控制力 $u$（$F_s=+u,\ F_{us}=-u$）
  - `model_node` 新增订阅 `control_force` 话题，未收到时控制力为 0（退化为被动悬架）
- **独立测试程序 `test_lqr`**
  - 新增 `src/test_lqr.cpp`，不依赖 rclcpp：构造 LQR → 初始条件响应仿真 → 对比开环/闭环衰减

### 配置变更

- `CMakeLists.txt`：新增 `controller_node`、`test_lqr` 目标，`lqr_controller.cpp` 源文件
- `package.xml`：版本号升至 `2.1.0`
- `config/model_params.yaml`：不变（物理参数与控制器默认一致）

### 文档更新

- `README.md`：新增 LQR 控制器说明、`controller_node` / `test_lqr` 运行方式、闭环验证结果
- `RUNNING.md`：新增闭环运行步骤（三节点联调）

### 验证结果（闭环对比）

| 指标 | 开环（被动） | 闭环（LQR） | 改善 |
| ---- | ------------ | ----------- | ---- |
| 车身加速度 RMS（m/s²） | 0.2947 | 0.2224 | **↓24.5%** |

> 闭环验证：`road_input_node` + `model_node` + `controller_node` 三节点联调，
> 正弦路面 0.05 m / 0.5 Hz，采样 `suspension_state.body_accel` 计算 RMS。

### 本次新掌握知识点

- 离散代数 Riccati 方程（DARE）不动点迭代求解；Eigen 无内置 Riccati 求解器，`care()` 是 MATLAB 接口且针对连续方程
- 悬架 LQR 应使用**悬架状态空间**（动行程 + 轮胎变形 + 速度）而非绝对位移，否则系统不完全可控（可控性矩阵秩 < n）导致 Riccati 无解
- 零阶保持（ZOH）离散化优于显式欧拉：`unsupported/Eigen/MatrixFunctions` 提供矩阵指数 `exp()`
- 显式欧拉在车身高频模态（~11 Hz）下数值不稳定，验证闭环需用离散状态方程或减小步长
- ROS 闭环联调：`model_node` 订阅 `control_force` 形成闭环，QoS 队列深度 10

---

## v1.3

### 新增功能

- **悬架模型抽象基类 `SuspensionModel`**
  - 新增 `include/suspension_sim/suspension_model.hpp`，定义纯虚接口：`reset()` / `update()` / `getState()` / `getBodyAccel()` / `stateSize()` / `name()`
  - `QuarterCarModel2DOF` 改为**公开继承**该基类，并补充 `stateSize()` / `name()` 实现
- **YAML 参数加载**
  - 新增 `include/suspension_sim/params_loader.hpp` + `src/params_loader.cpp`
  - `loadParams(yaml_file)`：从 YAML 读取物理参数（`ms`/`mus`/`ks`/`cs`/`kt`/`rate`/`type`），文件不存在或解析失败时回退默认值
  - `createModel(params)`：**工厂函数**，按 `type` 字段构造具体模型，未知类型回退到 `quarter_car_2dof`
  - 新增默认配置 `config/model_params.yaml`
- **`model_node` 使用基类指针**
  - 节点改为持有 `std::unique_ptr<SuspensionModel>`，具体模型由 `loadParams` + `createModel` 构造
  - 新增 `model_yaml` 参数与 `MODEL_YAML` 环境变量；YAML 作为默认值，ROS 参数仍可覆盖
- **独立测试程序 `test_model`**
  - 新增 `src/test_model.cpp`，不依赖 rclcpp：加载 YAML → 工厂构造模型 → 正弦路面仿真 5 秒 → 每 1 秒打印状态
  - 已实测通过（加载 `config/model_params.yaml` 并输出合理瞬态响应）

### 配置变更

- `CMakeLists.txt`：新增 `find_package(yaml-cpp)`、`params_loader.cpp` 源文件、`test_model` 目标、`config/model_params.yaml` 安装、编译期注入默认 YAML 路径
- `package.xml`：新增 `<depend>yaml-cpp-vendor</depend>`

### 文档更新

- `README.md`：补充抽象基类、YAML 参数加载、测试程序说明，更新依赖表与项目结构
- `RUNNING.md`：新增 `test_model` 运行步骤，更新目录结构
- `learning_notes.txt`：新增本次掌握的知识点（见下）

### 本次新掌握知识点

- 抽象基类 + 工厂模式：`std::unique_ptr<SuspensionModel>` 统一持有不同具体模型，扩展新模型只需新增派生类并注册工厂分支
- `yaml-cpp` 基本用法：`YAML::Load`、`node["key"].as<double>()`、`node["key"]` 存在性判断；ROS 2 Jazzy 中通过 `yaml-cpp-vendor` 依赖引入
- CMake 技巧：多个可执行目标共享同一源文件（`params_loader.cpp`）时需在 `add_executable` 中分别列出；通过 `target_compile_definitions` 注入编译期路径宏

---

## v1.2

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
