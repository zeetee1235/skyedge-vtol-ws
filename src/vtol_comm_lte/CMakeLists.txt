cmake_minimum_required(VERSION 3.8)
project(vtol_comm_lte)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  src/telemetry_node.py
  RENAME telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
