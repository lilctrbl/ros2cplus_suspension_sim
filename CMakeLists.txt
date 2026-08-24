cmake_minimum_required(VERSION 3.8)
project(suspension_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 编译期注入默认 YAML 配置路径（model_node / test_model 使用）
set(MODEL_YAML_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/config/model_params.yaml")

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SuspensionState.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(road_input_node src/road_input_node.cpp)
target_include_directories(road_input_node PRIVATE
  include)
ament_target_dependencies(road_input_node rclcpp rcl_interfaces std_msgs)

add_executable(road_display_node src/road_display_node.cpp)
target_include_directories(road_display_node PRIVATE
  include)
ament_target_dependencies(road_display_node rclcpp std_msgs)

add_executable(model_node
  src/model_node.cpp
  src/params_loader.cpp)
target_include_directories(model_node PRIVATE
  include)
target_compile_definitions(model_node PRIVATE
  MODEL_YAML_DEFAULT="${MODEL_YAML_DEFAULT}")
ament_target_dependencies(model_node rclcpp std_msgs)
# Eigen3 为纯头文件库，使用 CMake 导入目标链接
target_link_libraries(model_node Eigen3::Eigen yaml-cpp)
# 链接本包由 rosidl_generate_interfaces 生成的消息类型支持库目标
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(model_node "${cpp_typesupport_target}")

# LQR 控制器节点：订阅 suspension_state，发布 control_force
add_executable(controller_node
  src/controller_node.cpp
  src/lqr_controller.cpp)
target_include_directories(controller_node PRIVATE
  include)
ament_target_dependencies(controller_node rclcpp std_msgs)
target_link_libraries(controller_node Eigen3::Eigen)
rosidl_get_typesupport_target(controller_cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(controller_node "${controller_cpp_typesupport_target}")

# 独立测试程序：加载 YAML → 构造模型 → 仿真打印（不依赖 rclcpp）
add_executable(test_model
  src/test_model.cpp
  src/params_loader.cpp)
target_include_directories(test_model PRIVATE
  include)
target_compile_definitions(test_model PRIVATE
  MODEL_YAML="${MODEL_YAML_DEFAULT}")
target_link_libraries(test_model Eigen3::Eigen yaml-cpp)

# 独立测试程序：LQR 闭环仿真对比（不依赖 rclcpp）
add_executable(test_lqr
  src/test_lqr.cpp
  src/lqr_controller.cpp)
target_include_directories(test_lqr PRIVATE
  include)
target_link_libraries(test_lqr Eigen3::Eigen)

install(TARGETS road_input_node road_display_node model_node controller_node
  test_model test_lqr
  DESTINATION lib/${PROJECT_NAME})

# 安装默认 YAML 配置
install(FILES config/model_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
