# 文件：rviz_plugins/CMakeLists.txt
# 用途：构建并安装 RViz2 Goal3DTool 动态插件及其配置资源。
# 包含的类与函数：不定义 C++ 类或函数；编译 Pose3DTool/Goal3DTool 实现，
# 启用 Qt MOC，并通过 pluginlib_export_plugin_description_file 注册插件。
# 配置步骤：查找依赖 -> 配置 Qt -> 生成共享库 -> 注册插件 -> 安装资源。
cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)
# RViz 插件同时依赖 Qt、OGRE 封装、pluginlib 和 ROS 2。
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Q_OBJECT 需要 Qt MOC 自动生成元对象代码；QT_NO_KEYWORDS 避免 Qt 宏污染。
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
# 将交互基类与目标发布派生类编译成 RViz2 可动态加载的共享库。
add_library(${PROJECT_NAME} SHARED
  src/pose_tool.cpp
  src/goal_tool.cpp
  include/goal_tool.hpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs pluginlib rclcpp rviz_common rviz_rendering tf2 tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
# 注册 plugin_description.xml，使 RViz 工具列表能够发现 Goal3DTool。
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
