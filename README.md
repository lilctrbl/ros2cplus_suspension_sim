# suspension_sim

悬架系统仿真（ROS 2 包）。当前为项目骨架，基于 `ament_cmake` 构建。

## 环境要求

- **ROS 2**（Humble / 或其他基于 ament_cmake 的发行版）
- **CMake** >= 3.8
- C++ 编译器（GCC / Clang，支持 C++17）

## 依赖

| 包名      | 用途                     |
| --------- | ------------------------ |
| `rclcpp`  | ROS 2 C++ 客户端库       |
| `std_msgs`| 标准消息类型             |
| `Eigen3`  | 线性代数（悬架数学计算） |

## 构建

```bash
# 在工作空间根目录（如 ~/suspension_ws）下执行
colcon build --packages-select suspension_sim
```

构建完成后，如需在环境中使用：

```bash
source install/setup.bash
```

## 测试

```bash
colcon test --packages-select suspension_sim
```

## 项目结构

```
suspension_sim/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包清单（ROS 2）
├── include/
│   └── suspension_sim/     # 公共头文件
├── src/                    # 源码
└── LICENSE                 # Apache-2.0 许可证
```

## 许可

本项目基于 [Apache License 2.0](LICENSE) 许可。
