cmake_minimum_required(VERSION 3.8)
project(marl_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(predicted_risk_layer SHARED
  src/predicted_risk_layer.cpp
)

target_include_directories(predicted_risk_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(predicted_risk_layer PUBLIC cxx_std_17)

ament_target_dependencies(predicted_risk_layer
  rclcpp
  pluginlib
  nav2_costmap_2d
  geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d predicted_risk_layer_plugin.xml)

install(
  TARGETS predicted_risk_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES predicted_risk_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
