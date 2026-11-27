# Nav2 Waypoint Tools

## 軌跡からウェイポイントへの変換

```bash
ros2 run nav2_waypoint_tools trajectory_to_waypoints.py \
  --trajectory "/path/to/trajectory.ply" \
  --map-yaml "/path/to/map.yaml" \
  --output "/path/to/waypoints.yaml" \
  --waypoint-spacing 10.0 \
  --corner-angle-deg 20.0
```

## ウェイポイントの編集

Map Serverの起動
```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:="/path/to/map.yaml" 
```

Map Serverの有効化
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

Waypoint_editorの起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:="/path/to/map.yaml" \
  waypoints_file:="/path/to/waypoints.yaml" \
  nav2_params_file:="$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml"
```

RViz
```bash
rviz2
```

保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

---

## 大域経路の事前検証

Waypoint editor起動
```bash
ros2 launch nav2_waypoint_tools waypoint_editor.launch.py \
  map_yaml:="/path/to/map.yaml" \
  waypoints_file:="/path/to/pointcloud.ply"
```

RVizおよびMap server起動
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:="/path/to/map.yaml" \
  prior_3d_map:="/path/to/pointcloud.ply"
```

Waypoints保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

経路事前検証
```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route "/path/to/waypoints.yaml" \
  --map-yaml "/path/to/map.yaml" \
  --nav2-params "$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml"
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信
