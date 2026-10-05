# 文件：grid_path_searcher/CMakeLists.txt
# 用途：配置路径规划包的编译、依赖和安装规则。
# 包含的类与函数：无 C++ 类；使用 CMake 指令生成 demo_node、random_complex
# 和 occupancy_demo_node 三个可执行程序。
# 配置步骤：声明工程 -> 查找 ROS 2/Eigen 依赖 -> 建立目标 -> 安装目标、头文件、
# launch 与 data -> 调用 ament_package() 导出软件包。
cmake_minimum_required(VERSION 3.8)
project(grid_path_searcher)
# 开启常用告警，尽早发现类型、未使用变量和不规范语法问题。
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 查找三维/二维节点使用的 ROS 2 消息、TF、Eigen 和构建系统依赖。
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 三维规划器由 ROS 节点入口和 A*、JPS 搜索算法两个编译单元组成。
add_executable(demo_node src/demo_node.cpp src/graph_searcher.cpp)
target_compile_features(demo_node PUBLIC cxx_std_17)
target_include_directories(demo_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(demo_node geometry_msgs nav_msgs rclcpp sensor_msgs visualization_msgs)

# 随机点云地图生成器。
add_executable(random_complex src/random_complex_node.cpp)
target_compile_features(random_complex PUBLIC cxx_std_17)
target_include_directories(random_complex PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(random_complex rclcpp sensor_msgs)

# ROS 1 教程数据使用的二维 OccupancyGrid 兼容规划器。
add_executable(occupancy_demo_node src/occupancy_demo_node.cpp)
target_compile_features(occupancy_demo_node PUBLIC cxx_std_17)
target_include_directories(occupancy_demo_node PRIVATE include)
ament_target_dependencies(
  occupancy_demo_node geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs visualization_msgs)

# 可执行程序安装到 lib/<包名>，头文件与 launch/data 安装到共享目录。
install(TARGETS demo_node random_complex occupancy_demo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch data DESTINATION share/${PROJECT_NAME})
ament_package()
