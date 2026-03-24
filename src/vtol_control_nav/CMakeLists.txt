cmake_minimum_required(VERSION 3.8)
project(vtol_control_nav)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  src/waypoint_nav_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src/trajectory_planners
  src/precision_landing
  src/mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
