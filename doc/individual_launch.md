# Individual Launch Commands (Debugging)

統合起動とは併用しない．

## Ouster OS2-64

```bash
ros2 launch whill_nav2_bringup os2_64.launch.py
```

## Hokuyo URG

```bash
ros2 launch whill_nav2_bringup urg.launch.py
```

## SLAM (FAST-LIO2)

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=os2_64.yaml \
  rviz:=false
```

- 標準設定：`os2_64.yaml`
- 屋内環境設定：`os2_64_indoor.yaml`
- 屋外環境設定：`os2_64_outdoor.yaml`

## Static TF

```bash
ros2 launch whill_nav2_bringup static_tf.launch.py
```

## Nav2, Map Server
```bash
ros2 launch whill_nav2_bringup whill_nav2.launch.py \
  prior_2d_map:=<2D Map Path>
```

## RViz2

```bash
ros2 launch whill_nav2_bringup visualization.launch.py
```

## WHILL Driver

```bash
ros2 launch whill_bringup whill_launch.py
```

## Waypoint Navigation

```bash
python3 scripts/trajectory_waypoint_navigator.py \
  --trajectory < Odometry CSV File > \
  --map-yaml < 2D Map YAML File > \
  --nav2-params config/nav2_params.yaml \
  --start-index 150 \
  --end-index 350 \
  --waypoint-spacing 0.80 \
  --corner-angle-deg 15.0 \
  --clearance 0.45 \
  --plan-preview
```

安全確認後，`--execute`を指定すると，実走行を開始する．