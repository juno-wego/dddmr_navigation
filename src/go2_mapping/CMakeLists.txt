cmake_minimum_required(VERSION 3.8)
project(go2_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  script/odom_child_frame_relay.py
  script/pointcloud_frame_relay.py
  script/pointcloud_merger.py
  script/save_go2_map.py
  script/topic_stamp_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
