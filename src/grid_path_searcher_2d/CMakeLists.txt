# 文件：grid_path_searcher_2d/CMakeLists.txt
# 用途：配置二维规划包的算法库、节点编译、依赖和资源安装规则。
# 包含的类与函数：不直接定义 C++ 类/函数；生成 grid_searcher_2d 算法库、
# demo_node_2d、random_map_2d 两个程序及全算法 RViz 参数面板。
# 配置步骤：查找依赖 -> 编译算法库 -> 编译并链接节点 -> 安装头文件与资源。
cmake_minimum_required(VERSION 3.8)
project(grid_path_searcher_2d)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(grid_searcher_2d
  src/grid_map_2d.cpp
  src/grid_search_astar.cpp
  src/grid_search_jps.cpp
  src/grid_search_prm.cpp
  src/grid_search_rrt.cpp
  src/grid_search_rrt_star.cpp
  src/grid_search_kinodynamic_rrt_star.cpp
  src/grid_search_anytime_rrt_star.cpp
  src/grid_search_informed_rrt_star.cpp)
target_compile_features(grid_searcher_2d PUBLIC cxx_std_17)
target_include_directories(grid_searcher_2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(grid_searcher_2d
  geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

add_executable(demo_node_2d src/demo_node.cpp)
target_compile_features(demo_node_2d PUBLIC cxx_std_17)
target_link_libraries(demo_node_2d grid_searcher_2d)
ament_target_dependencies(demo_node_2d
  geometry_msgs nav_msgs rclcpp rcl_interfaces std_msgs visualization_msgs)

add_executable(random_map_2d src/random_map_node.cpp)
target_compile_features(random_map_2d PUBLIC cxx_std_17)
ament_target_dependencies(random_map_2d nav_msgs rclcpp)

# Qt MOC 为 Q_OBJECT 面板生成元对象代码，pluginlib 让 RViz2 动态发现面板。
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(${PROJECT_NAME}_rviz_panel SHARED
  src/rrt_star_panel.cpp
  include/grid_path_searcher_2d/rrt_star_panel.hpp)
target_compile_features(${PROJECT_NAME}_rviz_panel PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_rviz_panel PRIVATE QT_NO_KEYWORDS)
target_include_directories(${PROJECT_NAME}_rviz_panel PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_rviz_panel pluginlib rclcpp rviz_common)
target_link_libraries(${PROJECT_NAME}_rviz_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS grid_searcher_2d demo_node_2d random_map_2d ${PROJECT_NAME}_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_package()
