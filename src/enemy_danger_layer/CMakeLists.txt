cmake_minimum_required(VERSION 3.8)
project(enemy_danger_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Compile C++ plugin shared library
add_library(enemy_danger_layer_core SHARED src/enemy_danger_layer.cpp)

ament_target_dependencies(enemy_danger_layer_core
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# Export description XML so pluginlib can discover it
pluginlib_export_plugin_description_file(nav2_costmap_2d enemy_danger_layer_plugin.xml)

install(TARGETS enemy_danger_layer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES enemy_danger_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()