cmake_minimum_required(VERSION 3.8)
project(whill_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_watchdog.py
  scripts/odom_to_base_link.py
  scripts/system_status_announcer.py
  scripts/trajectory_waypoint_navigator.py
  scripts/twist_to_whill_joy.py
  scripts/urg_obstacle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/urg_ground_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(Eigen3 REQUIRED)

ament_package()
