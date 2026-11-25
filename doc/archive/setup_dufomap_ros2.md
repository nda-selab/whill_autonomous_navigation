# DUFOMap ROS2 setup manual

ビルド
```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws

colcon build --packages-select dufomap_ros2 \
    --cmake-args \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
    -DDUFOMAP_ROOT=$HOME/ws_dufomap/dufomap
    
source ~/ros2_ws/install/setup.bash
```

1st Terminal: Velodyne Driver
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

2nd Terminal: DUFOMap ROS2
```bash
ros2 launch dufomap_ros2 dufomap.launch.py
```

3rd Terminal: RViz2
```bash
rviz2
```

- Fixed Frame: velodyne
- /velodyne_points
- /dufomap/static_points
- /dufomap/dynamic_points

reliabilityは`Best Effort`に変更
