cmake_minimum_required(VERSION 3.8)
project(vtol_vision_yolo)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  src/yolo_detect_node.py
  RENAME yolo_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
