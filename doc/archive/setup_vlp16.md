# VLP-16 setup manual

Velodyneドライバのインストール
```bash
sudo apt update
sudo apt install -y ros-humble-velodyne
```

VLP-16の起動
```bash
source /opt/ros/humble/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

点群の受信確認
```bash
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic info /velodyne_points
ros2 topic hz /velodyne_points
```

