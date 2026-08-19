# suspension_sim

悬架系统仿真（ROS 2 包）。基于 `ament_cmake` 构建。

## 环境要求

- **ROS 2**（Jazzy / 或其他基于 ament_cmake 的发行版）
- **CMake** >= 3.8
- C++ 编译器（GCC / Clang，支持 C++17）

## 依赖

| 包名             | 用途                     |
| ---------------- | ------------------------ |
| `rclcpp`         | ROS 2 C++ 客户端库       |
| `rcl_interfaces` | 参数设置回调返回值类型   |
| `std_msgs`       | 标准消息类型             |
| `Eigen3`         | 线性代数（悬架数学计算） |
| `rosidl_default_generators` | 自定义消息代码生成 |

## 构建

```bash
# 在工作空间根目录（如 ~/suspension_ws）下执行
colcon build --packages-select suspension_sim
```

构建完成后，如需在环境中使用：

```bash
source install/setup.bash
```

## 运行

> 详细的编译运行步骤（含新手向说明）请见 [RUNNING.md](RUNNING.md)。

### 观察话题通信（发布 + 订阅）

需要 **两个终端**（都先 `source /opt/ros/jazzy/setup.bash` 和 `source ~/suspension_ws/install/setup.bash`）。

**终端 A：启动发布节点**

```bash
ros2 run suspension_sim road_input_node
```

节点会以 **100 Hz** 频率在话题 `road_height`（类型 `std_msgs/msg/Float64`）上发布路面高度。

**终端 B：启动订阅节点**

```bash
ros2 run suspension_sim road_display_node
```

订阅节点会持续打印收到的高度值：

```
[INFO] [road_display_node]: road_height = 0.0500 m
[INFO] [road_display_node]: road_height = 0.0499 m
...
```

### 验证

```bash
# 查看发布频率（应约 100 Hz）
ros2 topic hz road_height

# 查看数据内容
ros2 topic echo road_height
```

### 切换路面类型

节点内置多种路面模型（策略模式），可通过参数热切换：

```bash
# 运行时切换（sine / square / random / speed_bump 四选一）
ros2 param set /road_input_node road_profile square

# 或启动时指定
ros2 run suspension_sim road_input_node --ros-args -p road_profile:=random
```

支持的参数：

| 参数名          | 类型   | 默认值 | 说明                 |
| --------------- | ------ | ------ | -------------------- |
| `road_profile`  | string | `sine` | 路面类型             |
| `amplitude`     | double | `0.05` | 振幅（单位 m）       |
| `frequency`     | double | `0.5`  | 频率（单位 Hz）      |

## 测试

```bash
colcon test --packages-select suspension_sim
```

## 项目结构

```
suspension_sim/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包清单（ROS 2）
├── README.md               # 说明文档
├── RUNNING.md              # 编译运行指南（新手向）
├── version.md              # 版本历史
├── include/
│   └── suspension_sim/
│       ├── road_profile.hpp     # 路面模型抽象基类与多种实现（策略模式）
│       └── quarter_car_model.hpp# 二自由度四分之一车模型类（欧拉积分）
├── msg/
│   └── SuspensionState.msg      # 自定义消息：悬架系统状态
├── src/
│   ├── road_input_node.cpp      # 路面输入节点（100 Hz 发布 road_height）
│   ├── road_display_node.cpp    # 显示节点（订阅并打印 road_height）
│   └── model_node.cpp           # 模型节点（订阅路面→更新模型→发布状态）
└── LICENSE                 # Apache-2.0 许可证
```

## 路面模型

| 实现类               | 名称         | 说明                                 |
| -------------------- | ------------ | ------------------------------------ |
| `SineRoadProfile`    | `sine`       | 正弦路面 `y = A·sin(2π·f·t)`         |
| `SquareRoadProfile`  | `square`     | 方波路面，±A 跳变                    |
| `RandomRoadProfile`  | `random`     | 随机路面，按采样间隔随机跳变         |
| `SpeedBumpRoadProfile` | `speed_bump` | 减速带路面，单次三角凸起             |

新增路面时，只需在 `road_profile.hpp` 中继承 `RoadProfile` 实现 `compute_height()`，并在 `road_input_node.cpp` 的 `rebuild_profile_impl()` 中注册即可。

## 悬架模型（二自由度四分之一车）

`quarter_car_model.hpp` 中的 `QuarterCarModel2DOF` 类实现二自由度悬架模型，采用**显式欧拉积分**：

- 状态向量：$[x_s,\ x_{us},\ \dot{x}_s,\ \dot{x}_{us}]$（簧上/簧下位移与速度）
- 状态方程：
  - $m_s \ddot{x}_s = -k_s(x_s-x_{us}) - c_s(\dot{x}_s-\dot{x}_{us})$
  - $m_{us}\ddot{x}_{us} = k_s(x_s-x_{us}) + c_s(\dot{x}_s-\dot{x}_{us}) - k_t(x_{us}-r)$

### 模型节点运行

```bash
# 终端 A：发布路面
ros2 run suspension_sim road_input_node

# 终端 B：运行模型节点
ros2 run suspension_sim model_node

# 终端 C：观察状态输出
ros2 topic echo suspension_state --once
```

模型参数可通过 ROS 参数覆盖（启动时指定）：

```bash
ros2 run suspension_sim model_node --ros-args -p ms:=250.0 -p ks:=20000.0
```

| 参数名 | 类型 | 默认值 | 说明 |
| ------ | ---- | ------ | ---- |
| `ms` | double | `300.0` | 簧上质量 (kg) |
| `mus` | double | `40.0` | 簧下质量 (kg) |
| `ks` | double | `15000.0` | 悬架刚度 (N/m) |
| `cs` | double | `1000.0` | 悬架阻尼 (N·s/m) |
| `kt` | double | `200000.0` | 轮胎刚度 (N/m) |
| `rate` | double | `100.0` | 模型更新频率 (Hz) |

### 自定义消息 `SuspensionState.msg`

模型节点发布到 `suspension_state` 话题，字段包括：`time`、`road_height`、`xs`、`xus`、`xs_dot`、`xus_dot`、`body_accel`。后期可扩展（如悬架动行程、轮胎载荷）。

## 节点

| 节点 | 文件 | 作用 |
| ---- | ---- | ---- |
| `road_input_node` | `src/road_input_node.cpp` | 发布者：100 Hz 发布路面高度到 `road_height` |
| `road_display_node` | `src/road_display_node.cpp` | 订阅者：订阅 `road_height` 并打印高度值 |
| `model_node` | `src/model_node.cpp` | 订阅 `road_height`，更新 2DOF 悬架模型，发布 `suspension_state` |

## 版本

当前版本：**v1.2**，详见 [version.md](version.md)。

## 许可

本项目基于 [Apache License 2.0](LICENSE) 许可。
