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

### 启动路面输入节点

```bash
ros2 run suspension_sim road_input_node
```

节点会以 **100 Hz** 频率在话题 `road_height`（类型 `std_msgs/msg/Float64`）上发布路面高度。

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
├── version.md              # 版本历史
├── include/
│   └── suspension_sim/
│       └── road_profile.hpp  # 路面模型抽象基类与多种实现（策略模式）
├── src/
│   └── road_input_node.cpp   # 路面输入节点（100 Hz 发布 road_height）
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

## 版本

当前版本：**v1.1**，详见 [version.md](version.md)。

## 许可

本项目基于 [Apache License 2.0](LICENSE) 许可。
