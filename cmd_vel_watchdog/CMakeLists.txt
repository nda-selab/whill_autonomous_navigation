cmake_minimum_required(VERSION 3.8)
project(cmd_vel_watchdog)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
