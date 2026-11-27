cmake_minimum_required(VERSION 3.8)
project(nav2_waypoint_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_to_waypoints.py
  scripts/trajectory_waypoint_navigator.py
  scripts/waypoint_editor.py
  scripts/waypoint_route_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/waypoint_route_core.py
  scripts/route_watchdog_client.py
  scripts/route_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
