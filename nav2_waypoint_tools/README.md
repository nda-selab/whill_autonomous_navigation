# Nav2 Waypoint Tools

## 軌跡からウェイポイントへの変換

```bash
ros2 run nav2_waypoint_tools trajectory_to_waypoints.py \
  --trajectory <Trajectory PLY file> \
  --map-yaml <2D Map YAML file> \
  --output <Waypoint Output YAML file>\
  --waypoint-spacing 10.0 \
  --corner-angle-deg 20.0
```

## ウェイポイントの編集

Map Serverの起動
```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=<2D Map YAML file> 
```

Map Serverの有効化
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

Waypoint_editorの起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:=<2D Map YAML file>  \
  waypoints_file:=<Waypoint YAML file> 
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
  map_yaml:=< 2D Map YAML file > \
  waypoints_file:=< Waypoint YAML file >
```

RVizおよびMap server起動
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=< 2D Map YAML file > \
  prior_3d_map:=< 3D Map PLY or PCD file >
```

Waypoints保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

経路事前検証
```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route < Waypoint YAML file > \
  --map-yaml < 2D Map YAML file > \
  --nav2-params < Nav2 Parameter YAML file >
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信
