# WHILL Nav2 Bringup

Terminal 1: WHILL Nav2 Bringup
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=< 2D Map YAML File > \
  prior_3d_map:=< 3D Map PCD or PLY File >
```

起動後，RVizの「2D Pose Estimate」からロボットの初期位置・姿勢を設定する．
`map → odom`のTFが配信され，Nav2の自己位置が正常に取得できることを確認してから自律走行を開始する．

Terminal 2: Waypoint Route Runner
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route < Waypoints YAML File > \
  --map-yaml < 2D Map YAML File > \
  --nav2-params < Nav2 Parameter YAML File >
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信