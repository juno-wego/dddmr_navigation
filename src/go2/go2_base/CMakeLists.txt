cmake_minimum_required(VERSION 3.8)
project(go2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(go2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(./include/)

add_executable(go2_driver src/go2_driver.cc)
ament_target_dependencies(go2_driver 
                          go2_msgs 
                          rclcpp 
                          geometry_msgs 
                          sensor_msgs 
                          nav_msgs 
                          tf2
                          tf2_geometry_msgs
                          std_msgs
                          )
target_link_libraries(go2_driver unitree_sdk2)

add_executable(go2_motion_example src/go2_motion_example.cpp)
ament_target_dependencies(go2_motion_example 
                          go2_msgs 
                          rclcpp 
                          std_msgs
                          )

add_executable(go2_sdk_direct_example src/go2_sdk_direct_example.cpp)
target_link_libraries(go2_sdk_direct_example unitree_sdk2)

add_executable(go2_patrol src/go2_patrol.cpp)
target_link_libraries(go2_patrol unitree_sdk2)

add_executable(go2_camera_publisher src/go2_camera_publisher.cpp)
ament_target_dependencies(go2_camera_publisher
  rclcpp sensor_msgs cv_bridge camera_info_manager
)
target_link_libraries(go2_camera_publisher ${OpenCV_LIBS})

set_target_properties(go2_driver go2_sdk_direct_example go2_patrol PROPERTIES 
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS
  go2_driver
  go2_motion_example
  go2_sdk_direct_example
  go2_patrol
  go2_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
