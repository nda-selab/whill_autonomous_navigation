cmake_minimum_required(VERSION 3.8)
project(robot_system_monitor)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/system_status_announcer.py
  scripts/system_performance_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
