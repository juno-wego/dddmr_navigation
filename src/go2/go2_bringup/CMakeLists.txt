cmake_minimum_required(VERSION 3.8)
project(go2_bringup)

set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(go2_lidar_tf_tuner_panel SHARED
  src/lidar_tf_tuner_panel.cpp
)

target_include_directories(go2_lidar_tf_tuner_panel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(go2_lidar_tf_tuner_panel
  Qt5::Widgets
)

ament_target_dependencies(go2_lidar_tf_tuner_panel
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  tf2
  tf2_ros
)

target_compile_definitions(go2_lidar_tf_tuner_panel PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS go2_lidar_tf_tuner_panel
  EXPORT go2_lidar_tf_tuner_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(go2_lidar_tf_tuner_panel HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  tf2
  tf2_ros
)
ament_export_include_directories(include)
ament_export_libraries(go2_lidar_tf_tuner_panel)

ament_package()
