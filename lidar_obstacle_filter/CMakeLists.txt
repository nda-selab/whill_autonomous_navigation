cmake_minimum_required(VERSION 3.8)
project(lidar_obstacle_filter)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ouster_obstacle_filter.py
  scripts/urg_front_obstacle_filter.py
  scripts/urg_side_obstacle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/ouster_filter_core.py
  scripts/urg_ground_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
