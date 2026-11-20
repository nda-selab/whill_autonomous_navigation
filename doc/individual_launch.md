# Individual Launch Commands (Debugging)

統合起動とは併用しない．

1. Ouster OS2-64

```bash
ros2 launch whill_nav2_bringup os2_64.launch.py
```

2. Hokuyo URG

```bash
ros2 launch whill_nav2_bringup urg.launch.py
```

3. FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=os2_64.yaml \
  rviz:=false
```

- 標準設定：`os2_64.yaml`
- 屋内環境設定：`os2_64_indoor.yaml`
- 屋外環境設定：`os2_64_outdoor.yaml`

4. Static TF

```bash
ros2 launch whill_nav2_bringup static_tf.launch.py
```

5. PointCloud to LaserScan

```bash
ros2 launch whill_nav2_bringup pointcloud_to_laserscan.launch.py
```

6. Nav2, Map Server

```bash
ros2 launch whill_nav2_bringup whill_nav2.launch.py \
  prior_2d_map:=<2D Map Path>
```

7. RViz

```bash
ros2 launch whill_nav2_bringup visualization.launch.py
```

8. WHILL driver

```bash
ros2 launch whill_bringup whill_launch.py
```

## Waypoint Navigation

10. 実行

```bash
python3 scripts/trajectory_waypoint_navigator.py \
  --trajectory /home/selab/data/bikan_outdoor/odom.csv \
  --map-yaml /home/selab/ros2_ws/src/whill_nav2_bringup/maps/map.yaml \
  --nav2-params config/nav2_params.yaml \
  --start-index 150 \
  --end-index 350 \
  --waypoint-spacing 0.80 \
  --corner-angle-deg 15.0 \
  --clearance 0.45 \
  --plan-preview
```

安全確認後，`--execute`を指定すると，実走行を開始する．