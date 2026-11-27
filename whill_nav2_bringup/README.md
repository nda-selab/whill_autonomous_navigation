# WHILL Nav2 Bringup

Terminal 1: WHILL Nav2 Bringup
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:="$HOME/data/map.yaml" \
  prior_3d_map:="$HOME/data/pointcloud.ply"
```

起動後，RVizの「2D Pose Estimate」からロボットの初期位置・姿勢を設定する．
`map → odom`のTFが配信され，Nav2の自己位置が正常に取得できることを確認してから自律走行を開始する．

Terminal 2: Waypoint Route Runner
```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route "$HOME/data/waypoints.yaml" \
  --map-yaml "$HOME/data/map.yaml" \
  --nav2-params "$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml"
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信