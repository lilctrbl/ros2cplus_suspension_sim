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
find_package(rosidl_default_generators REQUIRED)

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

add_executable(model_node src/model_node.cpp)
target_include_directories(model_node PRIVATE
  include)
ament_target_dependencies(model_node rclcpp std_msgs)
# Eigen3 为纯头文件库，使用 CMake 导入目标链接
target_link_libraries(model_node Eigen3::Eigen)
# 链接本包由 rosidl_generate_interfaces 生成的消息类型支持库目标
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(model_node "${cpp_typesupport_target}")

install(TARGETS road_input_node road_display_node model_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
