# 文件：waypoint_generator/CMakeLists.txt
# 用途：配置 waypoint_generator 节点的依赖、编译和安装规则。
# 包含的类与函数：无 C++ 类；使用 CMake 指令编译 WaypointGenerator 所在的
# waypoint_generator 可执行程序，并安装可复用的 sample_waypoints.hpp。
# 配置步骤：声明工程 -> 查找消息/TF 依赖 -> 建立目标 -> 安装并导出软件包。
cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)
# 航点生成器只生成一个 ROS 2 可执行程序。
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 声明消息、rclcpp 与 TF 转换依赖。
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 编译节点，并让编译器可以找到 include/sample_waypoints.hpp。
add_executable(waypoint_generator src/waypoint_generator.cpp)
target_compile_features(waypoint_generator PUBLIC cxx_std_17)
target_include_directories(waypoint_generator PRIVATE include)
ament_target_dependencies(waypoint_generator geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs)
# 安装运行程序和可复用头文件。
install(TARGETS waypoint_generator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
